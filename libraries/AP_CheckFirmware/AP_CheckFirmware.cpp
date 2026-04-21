/*
  support checking board ID and firmware CRC in the bootloader
 */
#include "AP_CheckFirmware.h"
#include <AP_HAL/HAL.h>
#include <AP_Math/crc.h>

#if AP_CHECK_FIRMWARE_ENABLED

#if defined(HAL_BOOTLOADER_BUILD)

#if AP_SIGNED_FIRMWARE
#include "../../Tools/AP_Bootloader/support.h"
#include <string.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#include "../../modules/mavlink/pymavlink/generator/C/include_v2.0/mavlink_sha256.h"
#pragma GCC diagnostic pop
#include "monocypher.h"

const struct ap_secure_data public_keys __attribute__((section(".apsec_data")));

/*
  return true if all public keys are zero. We allow boot of an
  unsigned firmware in that case
 */
static bool all_zero_public_keys(void)
{
    /*
      look over all public keys, if one matches then we are OK
     */
    const uint8_t zero_key[AP_PUBLIC_KEY_LEN] {};
    for (const auto &public_key : public_keys.public_key) {
        if (memcmp(public_key.key, zero_key, AP_PUBLIC_KEY_LEN) != 0) {
            return false;
        }
    }
    return true;
}

/*
  check a signature against bootloader keys
 */

/*
  The checksum registry is stored in protected bootloader flash placed by the
  linker in the .apsec_data section alongside ap_secure_data.
*/
const struct ap_secure_checksum_registry checksum_registry __attribute__((section(".apsec_data"))) = {};

/*
  Return true if the checksum registry sig/algo/version fields are valid.
*/
static bool checksum_registry_valid(void)
{
    const uint8_t expected_sig[8] = AP_CHECKSUM_REGISTRY_SIGNATURE;
    return (memcmp(checksum_registry.sig, expected_sig, sizeof(expected_sig)) == 0 &&
            checksum_registry.algo == AP_CHECKSUM_REGISTRY_ALGO_SHA2_256 &&
            checksum_registry.version == 1);
}

/*
  Finalize MAVLink SHA2-256 and return all 32 bytes.
 */
static void mavlink_sha256_final_256(mavlink_sha256_ctx *m, uint8_t result[32])
{
    uint32_t bit_length;
    unsigned offset = m->sz % 64;

    bit_length = m->sz * 8;

    m->u.save_bytes[offset++] = 0x80;
    if (offset > 56) {
        memset(&m->u.save_bytes[offset], 0, 64-offset);
        mavlink_sha256_calc(m);
        offset = 0;
    }
    memset(&m->u.save_bytes[offset], 0, 60-offset);
    m->u.save_bytes[60] = (bit_length >> 24) & 0xFF;
    m->u.save_bytes[61] = (bit_length >> 16) & 0xFF;
    m->u.save_bytes[62] = (bit_length >> 8) & 0xFF;
    m->u.save_bytes[63] = (bit_length >> 0) & 0xFF;
    mavlink_sha256_calc(m);

    for (uint8_t i = 0; i < 8; i++) {
        const uint32_t c = m->counter[i];
        result[i*4 + 0] = (c >> 24) & 0xFF;
        result[i*4 + 1] = (c >> 16) & 0xFF;
        result[i*4 + 2] = (c >> 8) & 0xFF;
        result[i*4 + 3] = (c >> 0) & 0xFF;
    }
}

/*
  Compute a 32-byte SHA2-256 hash over up to two contiguous flash regions.
  region2/len2 may be nullptr/0 to hash a single region.
*/
static void compute_partition_hash(const uint8_t *region1, uint32_t len1,
                                   const uint8_t *region2, uint32_t len2,
                                   uint8_t hash_out[32])
{
    mavlink_sha256_ctx ctx;
    mavlink_sha256_init(&ctx);
    mavlink_sha256_update(&ctx, region1, len1);
    if (region2 != nullptr && len2 > 0) {
        mavlink_sha256_update(&ctx, region2, len2);
    }
    mavlink_sha256_final_256(&ctx, hash_out);
}

static check_fw_result_t check_firmware_signature(const app_descriptor_signed *ad,
                                                  const uint8_t *flash1, uint32_t len1,
                                                  const uint8_t *flash2, uint32_t len2)
{
#ifndef SECURITY_STRICT_PROD
    if (all_zero_public_keys()) {
        return check_fw_result_t::CHECK_FW_OK;
    }
#endif // SECURITY_STRICT_PROD

    // 8 byte signature version
    static const uint64_t sig_version = 30437LLU;
    if (ad->signature_length != 72) {
        return check_fw_result_t::FAIL_REASON_BAD_FIRMWARE_SIGNATURE;
    }
    if (memcmp((const uint8_t*)&sig_version, ad->signature, sizeof(sig_version)) != 0) {
        return check_fw_result_t::FAIL_REASON_BAD_FIRMWARE_SIGNATURE;
    }

    /*
      look over all public keys, if one matches then we are OK
     */
    for (const auto &public_key : public_keys.public_key) {
        crypto_check_ctx ctx {};
        crypto_check_ctx_abstract *actx = (crypto_check_ctx_abstract*)&ctx;
        crypto_check_init(actx, &ad->signature[sizeof(sig_version)], public_key.key);

        crypto_check_update(actx, flash1, len1);
        crypto_check_update(actx, flash2, len2);
        if (crypto_check_final(actx) == 0) {
            // good signature
            return check_fw_result_t::CHECK_FW_OK;
        }
    }

    // none of the public keys matched
    return check_fw_result_t::FAIL_REASON_VERIFICATION;
}
#endif // AP_SIGNED_FIRMWARE

/*
  check firmware CRC and board ID to see if it matches
 */
static check_fw_result_t check_good_firmware_signed(void)
{
    const uint8_t sig[8] = AP_APP_DESCRIPTOR_SIGNATURE_SIGNED;
    const uint8_t *flash1 = (const uint8_t *)(FLASH_LOAD_ADDRESS + (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB)*1024);
    const uint32_t flash_size = (BOARD_FLASH_SIZE - (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB))*1024;
    const app_descriptor_signed *ad = (const app_descriptor_signed *)memmem(flash1, flash_size-sizeof(app_descriptor_signed), sig, sizeof(sig));
    if (ad == nullptr) {
        // no application signature
        return check_fw_result_t::FAIL_REASON_NO_APP_SIG;
    }
    // check length
    if (ad->image_size > flash_size) {
        return check_fw_result_t::FAIL_REASON_BAD_LENGTH_APP;
    }

    bool id_ok = (ad->board_id == APJ_BOARD_ID);
#ifdef ALT_BOARD_ID
    id_ok |= (ad->board_id == ALT_BOARD_ID);
#endif

    if (!id_ok) {
        return check_fw_result_t::FAIL_REASON_BAD_BOARD_ID;
    }

    const uint8_t *flash2 = (const uint8_t *)&ad->version_major;
    const uint8_t desc_len = offsetof(app_descriptor_signed, version_major) - offsetof(app_descriptor_signed, image_crc1);
    const uint32_t len1 = ((const uint8_t *)&ad->image_crc1) - flash1;

    if ((len1 + desc_len) > ad->image_size) {
        return check_fw_result_t::FAIL_REASON_BAD_LENGTH_DESCRIPTOR;
    }

    const uint32_t len2 = ad->image_size - (len1 + desc_len);
    uint32_t crc1 = crc32_small(0, flash1, len1);
    uint32_t crc2 = crc32_small(0, flash2, len2);
    if (crc1 != ad->image_crc1 || crc2 != ad->image_crc2) {
        return check_fw_result_t::FAIL_REASON_BAD_CRC;
    }

    check_fw_result_t ret = check_fw_result_t::CHECK_FW_OK;

#if AP_SIGNED_FIRMWARE
    ret = check_firmware_signature(ad, flash1, len1, flash2, len2);
    if (ret != check_fw_result_t::CHECK_FW_OK) {
        return ret;
    }

    // Verify code and data partition hashes against the protected registry if present.
    if (checksum_registry_valid()) {
        uint8_t computed_code[32];
        uint8_t computed_data[32];
        // code partition: flash1 region (firmware start up to image_crc1 field)
        compute_partition_hash(flash1, len1, nullptr, 0, computed_code);
        // data partition: flash2 region (version_major through end of image)
        compute_partition_hash(flash2, len2, nullptr, 0, computed_data);

        if (memcmp(computed_code, checksum_registry.code_hash, 32) != 0 ||
            memcmp(computed_data, checksum_registry.data_hash, 32) != 0) {
            return check_fw_result_t::FAIL_REASON_BAD_PARTITION_HASH;
        }
    }
#endif

    return ret;
}

/*
  check firmware CRC and board ID to see if it matches, using unsigned
  signature
 */
static check_fw_result_t check_good_firmware_unsigned(void)
{
    const uint8_t sig[8] = AP_APP_DESCRIPTOR_SIGNATURE_UNSIGNED;
    const uint8_t *flash1 = (const uint8_t *)(FLASH_LOAD_ADDRESS + (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB)*1024);
    const uint32_t flash_size = (BOARD_FLASH_SIZE - (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB))*1024;
    const app_descriptor_unsigned *ad = (const app_descriptor_unsigned *)memmem(flash1, flash_size-sizeof(app_descriptor_unsigned), sig, sizeof(sig));
    if (ad == nullptr) {
        // no application signature
        return check_fw_result_t::FAIL_REASON_NO_APP_SIG;
    }
    // check length
    if (ad->image_size > flash_size) {
        return check_fw_result_t::FAIL_REASON_BAD_LENGTH_APP;
    }

    bool id_ok = (ad->board_id == APJ_BOARD_ID);
#ifdef ALT_BOARD_ID
    id_ok |= (ad->board_id == ALT_BOARD_ID);
#endif

    if (!id_ok) {
        return check_fw_result_t::FAIL_REASON_BAD_BOARD_ID;
    }

    const uint8_t *flash2 = (const uint8_t *)&ad->version_major;
    const uint8_t desc_len = offsetof(app_descriptor_unsigned, version_major) - offsetof(app_descriptor_unsigned, image_crc1);
    const uint32_t len1 = ((const uint8_t *)&ad->image_crc1) - flash1;

    if ((len1 + desc_len) > ad->image_size) {
        return check_fw_result_t::FAIL_REASON_BAD_LENGTH_DESCRIPTOR;
    }

    const uint32_t len2 = ad->image_size - (len1 + desc_len);
    uint32_t crc1 = crc32_small(0, flash1, len1);
    uint32_t crc2 = crc32_small(0, flash2, len2);
    if (crc1 != ad->image_crc1 || crc2 != ad->image_crc2) {
        return check_fw_result_t::FAIL_REASON_BAD_CRC;
    }

    return check_fw_result_t::CHECK_FW_OK;
}

check_fw_result_t check_good_firmware(void)
{
#if AP_SIGNED_FIRMWARE
    // allow unsigned format if we have no public keys. This allows
    // for use of SECURE_COMMAND to remove all public keys and then
    // load of unsigned firmware
    const auto ret = check_good_firmware_signed();
    if (ret != check_fw_result_t::CHECK_FW_OK &&
        all_zero_public_keys() &&
        check_good_firmware_unsigned() == check_fw_result_t::CHECK_FW_OK) {
        return check_fw_result_t::CHECK_FW_OK;
    }
    return ret;
#else
    const auto ret = check_good_firmware_unsigned();
    if (ret != check_fw_result_t::CHECK_FW_OK) {
        // allow for signed format, not checking public keys. This
        // allows for booting of a signed firmware with an unsigned
        // bootloader, which allows for bootstrapping a system up from
        // unsigned to signed
        const auto ret2 = check_good_firmware_signed();
        if (ret2 == check_fw_result_t::CHECK_FW_OK) {
            return check_fw_result_t::CHECK_FW_OK;
        }
    }
    return ret;
#endif
}

const app_descriptor_t *get_app_descriptor(void)
{
#if AP_SIGNED_FIRMWARE
    const uint8_t sig[8] = AP_APP_DESCRIPTOR_SIGNATURE_SIGNED;
#else
    const uint8_t sig[8] = AP_APP_DESCRIPTOR_SIGNATURE_UNSIGNED;
#endif
    const uint8_t *flash1 = (const uint8_t *)(FLASH_LOAD_ADDRESS + (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB)*1024);
    const uint32_t flash_size = (BOARD_FLASH_SIZE - (FLASH_BOOTLOADER_LOAD_KB + APP_START_OFFSET_KB))*1024;
    const app_descriptor_t *ad = (const app_descriptor_t *)memmem(flash1, flash_size-sizeof(app_descriptor_t), sig, sizeof(sig));
    return ad;
}

#endif // HAL_BOOTLOADER_BUILD

#if !defined(HAL_BOOTLOADER_BUILD)
extern const AP_HAL::HAL &hal;
extern const app_descriptor_t app_descriptor;

/*
  this is needed to ensure we don't elide the app_descriptor
 */
void check_firmware_print(void)
{
    hal.console->printf("Booting %u/%u\n",
                        app_descriptor.version_major,
                        app_descriptor.version_minor);
}
#endif


#endif // AP_CHECK_FIRMWARE_ENABLED
