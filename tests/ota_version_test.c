#include <assert.h>
#include "../src/device_drv/ota_upgrade/ota_ecu_update.c"
static int test_access(const char *path, int mode);
#define access test_access
#include "../src/function_task/ota_task/otaupgrad_task.c"
#undef access

XCPStatus xcpstatus;
UpgradeInfo g_max_upgrade;

static const char *test_filename;
static unsigned char test_device_type;
static int unpack_calls;
static uint16_t ota_status;
static int source_exists = 1;
static int storage_result;
static int copy_result = 1;
static int copy_calls;
static int storage_calls;
static int unpack_result = -1;

static int test_access(const char *path, int mode)
{
    char expected[512];
    snprintf(expected, sizeof(expected), "%s/%s", OTA_INCOMING_PATH, test_filename);
    assert(strcmp(path, expected) == 0);
    assert(mode == F_OK);
    return source_exists ? 0 : -1;
}

const char *get_ota_OTAFilename(void) { return test_filename; }
unsigned int get_ota_deviceID(void) { return 0; }
unsigned char get_ota_deviceType(void) { return test_device_type; }
unsigned char get_ota_OTAStart(void) { return 1; }

int set_modbus_reg_val(uint16_t address, uint16_t value)
{
    if (address == OTASTATUSREGADDR) {
        ota_status = value;
    }
    return 0;
}

void log_output(const char *file, size_t filelen, const char *func,
                size_t funclen, long line, const char *format, ...)
{
    (void)file;
    (void)filelen;
    (void)func;
    (void)funclen;
    (void)line;
    (void)format;
}

/* Keep all simulated file operations away from the device OTA directories. */
bool ota_filename_is_safe(const char *filename)
{
    return filename != NULL && filename[0] != '\0';
}

int unzipfile(char *path, unsigned int *error, file_type_t type)
{
    assert(type == FILE_TYPE_BIN || test_device_type == ECU);
    if (test_device_type != ECU) {
        assert(strcmp(path, OTA_READY_PATH) == 0);
    }
    unpack_calls++;
    *error = unpack_result < 0 ? OTA_ERR_SOURCE_PACKAGE_MISSING : 0;
    return unpack_result;
}

int ensure_ota_storage_dirs(void)
{
    storage_calls++;
    return storage_result;
}

int copy_file(const char *source, const char *destination)
{
    char expected[512];
    snprintf(expected, sizeof(expected), "%s/%s", OTA_INCOMING_PATH, test_filename);
    assert(strcmp(source, expected) == 0);
    snprintf(expected, sizeof(expected), "%s/%s", OTA_READY_PATH, test_filename);
    assert(strcmp(destination, expected) == 0);
    copy_calls++;
    return copy_result;
}

static void check_ecu(const char *filename, int allowed)
{
    test_filename = filename;
    test_device_type = ECU;
    unpack_calls = 0;
    ota_status = OTAIDLE;
    ECU_OTA();
    assert(unpack_calls == allowed);
    assert(ecustatus.ErrorReg == (allowed ? OTA_ERR_SOURCE_PACKAGE_MISSING
                                         : OTA_ERR_VERSION_MISMATCH));
    assert(ota_status == OTAFAILED);
}

static void check_bcu(const char *filename, int allowed)
{
    test_filename = filename;
    test_device_type = BCU;
    unpack_calls = 0;
    /* The unpack stub stops accepted filenames before any real upgrade. */
    assert(precheck_bcu_bmu_ota_package() == -1);
    assert(unpack_calls == allowed);
    assert(xcpstatus.ErrorReg == (allowed ? OTA_ERR_SOURCE_PACKAGE_MISSING
                                         : OTA_ERR_VERSION_MISMATCH));
}

static void check_direct_bin(unsigned char device_type, const char *filename,
                             int expected_error)
{
    test_device_type = device_type;
    test_filename = filename;
    unpack_calls = copy_calls = storage_calls = 0;
    memset(&g_max_upgrade, 0x5a, sizeof(g_max_upgrade));
    xcpstatus.ErrorReg = OTA_ERR_UPGRADE_FILE_OPEN_FAILED;
    assert(precheck_bcu_bmu_ota_package() == (expected_error ? -1 : 0));
    assert(xcpstatus.ErrorReg == (unsigned int)expected_error);
    assert(unpack_calls == 0);
    assert(storage_calls == source_exists);
    assert(copy_calls == (source_exists && storage_result == 0));
    if (expected_error == 0) {
        assert(strcmp(g_max_upgrade.upgrade_file, filename) == 0);
        assert(g_max_upgrade.md5sum[0] == '\0');
    }
}

int main(void)
{
    char filename[64];
    for (char digit = '0'; digit <= '9'; digit++) {
        snprintf(filename, sizeof(filename), "XC_ECU_V%c01.tar", digit);
        check_ecu(filename, digit != '2');
        snprintf(filename, sizeof(filename), "XC_BCU_V%c01.tar", digit);
        check_bcu(filename, digit != '3');
    }

    check_ecu(NULL, 0);
    check_ecu("", 0);
    check_ecu("XC_BCU_V101.tar", 0);
    check_ecu("XC_ECU_101.tar", 0);
    check_ecu("XC_ECU_V", 0);
    check_ecu("XC_ECU_V.tar", 0);
    check_ecu("XC_ECU_Vx01.tar", 0);
    check_ecu("XC_ECU_V201_V101.tar", 0);
    check_ecu("XC_ECU_V101_V201.tar", 1);
    check_bcu("XC_ECU_V501.tar", 0);
    check_bcu("XC_BCU_501.tar", 0);
    check_bcu("XC_BCU_V", 0);
    check_bcu("XC_BCU_V.tar", 0);
    check_bcu("XC_BCU_Vx01.tar", 0);
    check_bcu("XC_BCU_V301_V501.tar", 0);
    check_bcu("XC_BCU_V501_V301.tar", 1);

    for (int device = 0; device < 2; device++) {
        unsigned char type = device ? BMU : BCU;
        const char *name = device ? "XC_BMU_V301.bin" : "XC_BCU_V501.bin";
        check_direct_bin(type, name, 0);
        source_exists = 0;
        check_direct_bin(type, name, OTA_ERR_SOURCE_PACKAGE_MISSING);
        source_exists = 1;
        storage_result = -1;
        check_direct_bin(type, name, OTA_ERR_COPY_TARGET_FAILED);
        storage_result = 0;
        copy_result = 0;
        check_direct_bin(type, name, OTA_ERR_COPY_TARGET_FAILED);
        copy_result = 1;
        check_direct_bin(type, name, 0);
    }
    for (char digit = '0'; digit <= '9'; digit++) {
        snprintf(filename, sizeof(filename), "XC_BMU_V%c01.bin", digit);
        check_direct_bin(BMU, filename, 0);
    }

    test_device_type = BMU;
    test_filename = "XC_BMU_V301.tar";
    unpack_calls = copy_calls = 0;
    assert(precheck_bcu_bmu_ota_package() == -1);
    assert(unpack_calls == 1 && copy_calls == 0);
    assert(xcpstatus.ErrorReg == OTA_ERR_SOURCE_PACKAGE_MISSING);
    unpack_result = 1;
    assert(precheck_bcu_bmu_ota_package() == 0);
    assert(unpack_calls == 2 && copy_calls == 0);
    assert(xcpstatus.ErrorReg == 0);
    puts("OTA version and package precheck tests passed");
    return 0;
}
