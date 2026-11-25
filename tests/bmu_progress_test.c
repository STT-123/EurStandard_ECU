#include <assert.h>
#include "../src/device_drv/ota_upgrade/ota_xcp_update.c"

static uint16_t reported_progress;
static unsigned int writes;
static unsigned int errors;
static unsigned int seen[100];

int set_modbus_reg_val(uint16_t address, uint16_t value)
{
    assert(address == OTAPPROGRESSREGADDR);
    assert(value <= 99);
    if (value != 0) {
        assert(value > reported_progress);
    }
    reported_progress = value;
    seen[value]++;
    writes++;
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
    errors++;
}

static void check_transfer(unsigned int filesize)
{
    memset(seen, 0, sizeof(seen));
    writes = 0;
    bmu_ota_progress_begin(15);
    for (unsigned int device = 0; device < 15; device++) {
        bmu_ota_progress_set_device(device);
        for (unsigned int bytes = 7; bytes < filesize; bytes += 7) {
            update_bmu_ota_progress(bytes, filesize);
            assert(reported_progress ==
                   ((uint64_t)device * filesize * 100 + (uint64_t)bytes * 100) /
                   ((uint64_t)filesize * 15));
        }
        update_bmu_ota_progress(filesize, filesize);
        bmu_ota_progress_complete_device();
    }
    assert(reported_progress == 99);
    assert(writes == 100);
    for (unsigned int percent = 0; percent < 100; percent++) {
        assert(seen[percent] == 1);
    }
}

int main(void)
{
    check_transfer(210688); /* Includes a partial final CAN packet. */
    check_transfer(210686); /* Exact multiple of seven bytes. */

    bmu_ota_progress_begin(15);
    update_bmu_ota_progress(50, 100);
    assert(reported_progress == 3);
    unsigned int previous_writes = writes;
    update_bmu_ota_progress(0, 100);
    update_bmu_ota_progress(20, 100);
    assert(reported_progress == 3 && writes == previous_writes);
    update_bmu_ota_progress(80, 100);
    assert(reported_progress == 5);
    bmu_ota_progress_complete_device();
    assert(reported_progress == 6);
    bmu_ota_progress_set_device(1);
    update_bmu_ota_progress(10, 100);
    assert(reported_progress == 7);

    previous_writes = writes;
    update_bmu_ota_progress(1, 0);
    update_bmu_ota_progress(101, 100);
    bmu_ota_progress_set_device(15);
    bmu_ota_progress_complete_device();
    assert(errors == 3 && writes == previous_writes);

    bmu_ota_progress_begin(15);
    assert(reported_progress == 0);
    bmu_ota_progress_set_device(14);
    bmu_ota_progress_complete_device();
    assert(reported_progress == 99);
    puts("BMU progress tests passed");
    return 0;
}
