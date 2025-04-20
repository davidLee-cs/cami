
#include "accelerometer.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"

/******************************************************************************/
/*!                Macro definition                                           */

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)


float cami_accel[3] = {0.0f, };


/******************************************************************************/
/*!           Static Function Declaration                                     */

/*!
 * @brief This internal API converts raw sensor values(LSB) to meters per seconds square.
 *
 *  @param[in] val       : Raw sensor value.
 *  @param[in] g_range   : Accel Range selected (2G, 4G, 8G, 16G).
 *  @param[in] bit_width : Resolution of the sensor.
 *
 *  @return Accel values in meters per second square.
 *
 */
static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width);

    int8_t rslt;
    uint8_t sensor_ctrl;
    uint8_t n_conf = 1;
    uint8_t n_status = 1;
    uint8_t loop = 0;
    uint8_t limit = 50;
    float x = 0, y = 0, z = 0;

    struct bma5_acc_conf acc_cfg;
    struct bma530_int_map int_map;
    struct bma5_accel sens_data;
    struct bma5_sensor_status status;
    struct bma5_int_conf_types int_config;
    struct bma530_int_status_types int_status;
    struct bma5_dev dev;

/******************************************************************************/
void bma530Accel_init(void)
{
    // int8_t rslt;
    // uint8_t sensor_ctrl;
    // uint8_t n_conf = 1;
    // uint8_t n_status = 1;
    // uint8_t loop = 0;
    // uint8_t limit = 50;
    // float x = 0, y = 0, z = 0;

    // struct bma5_acc_conf acc_cfg;
    // struct bma530_int_map int_map;
    // struct bma5_accel sens_data;
    // struct bma5_sensor_status status;
    // struct bma5_int_conf_types int_config;
    // struct bma530_int_status_types int_status;
    // struct bma5_dev dev;

    /* Assign context parameter selection */
    enum bma5_context context;

    context = BMA5_SMARTPHONE;

    int_config.int_src = BMA5_INT_1;
    int_status.int_src = BMA530_INT_STATUS_INT1;

    /* Interface reference is given as a parameter
     *         For I2C : BMA5_I2C_INTF
     *         For SPI : BMA5_SPI_INTF
     */
    rslt = bma5_interface_init(&dev, BMA5_I2C_INTF, context);
    // bma5_check_rslt("bma5_interface_init", rslt);

    rslt = bma530_init(&dev);
    // bma5_check_rslt("bma530_init", rslt);
    // printf("Chip ID:0x%x\n\n", dev.chip_id);

    /* Map generic interrupts to hardware interrupt pin of the sensor */
    rslt = bma530_get_int_map(&int_map, &dev);
    // bma5_check_rslt("bma530_get_int_map", rslt);

    /* Map accel data ready interrupt */
    int_map.acc_drdy_int_map = BMA530_ACC_DRDY_INT_MAP_INT1;
    rslt = bma530_set_int_map(&int_map, &dev);
    // bma5_check_rslt("bma530_set_int_map", rslt);

    /* Get accel configurations */
    rslt = bma5_get_acc_conf_0(&sensor_ctrl, &dev);
    // bma5_check_rslt("bma5_get_acc_conf_0", rslt);

    rslt = bma5_get_acc_conf(&acc_cfg, &dev);
    // bma5_check_rslt("bma5_get_acc_conf", rslt);

    /* Set accel configurations */
    acc_cfg.acc_odr = BMA5_ACC_ODR_HZ_25;
    acc_cfg.acc_bwp = BMA5_ACC_BWP_NORM_AVG4;
    acc_cfg.power_mode = BMA5_POWER_MODE_HPM;

    acc_cfg.acc_range = BMA5_ACC_RANGE_MAX_2G;
    acc_cfg.acc_iir_ro = BMA5_ACC_IIR_RO_DB_40;
    acc_cfg.noise_mode = BMA5_NOISE_MODE_LOWER_POWER;
    acc_cfg.acc_drdy_int_auto_clear = BMA5_ACC_DRDY_INT_AUTO_CLEAR_ENABLED;

    rslt = bma5_set_acc_conf(&acc_cfg, &dev);
    // bma5_check_rslt("bma5_set_acc_conf", rslt);

    // printf("\nAccel configurations\n");
    // printf("ODR : %s\t\n", enum_to_string(BMA5_ACC_ODR_HZ_25));
    // printf("BWP : %s\t\n", enum_to_string(BMA5_ACC_BWP_NORM_AVG4));
    // printf("Power mode : %s\t\n", enum_to_string(BMA5_POWER_MODE_HPM));
    // printf("Range : %s\t\n", enum_to_string(BMA5_ACC_RANGE_MAX_2G));
    // printf("IIR filter : %s\t\n", enum_to_string(BMA5_ACC_IIR_RO_DB_40));
    // printf("Noise mode : %s\t\n", enum_to_string(BMA5_NOISE_MODE_LOWER_POWER));
    // printf("Data ready interrupt auto clear : %s\t\n", enum_to_string(BMA5_ACC_DRDY_INT_AUTO_CLEAR_ENABLED));

    /* Enable accel */
    sensor_ctrl = BMA5_SENSOR_CTRL_ENABLE;

    rslt = bma5_set_acc_conf_0(sensor_ctrl, &dev);
    // bma5_check_rslt("bma5_set_acc_conf_0", rslt);

    if (rslt == BMA5_OK)
    {
        // printf("\nAccel enabled\n");
    }

    /* Map hardware interrupt pin configurations */
    rslt = bma5_get_int_conf(&int_config, n_conf, &dev);
    // bma5_check_rslt("bma5_get_int_conf", rslt);

    int_config.int_conf.int_mode = BMA5_INT1_MODE_LATCHED;
    int_config.int_conf.int_od = BMA5_INT1_OD_PUSH_PULL;
    int_config.int_conf.int_lvl = BMA5_INT1_LVL_ACTIVE_HIGH;

    rslt = bma5_set_int_conf(&int_config, n_conf, &dev);
    // bma5_check_rslt("bma5_set_int_conf", rslt);

    // printf("\nInterrupt configurations\n");
    // printf("Int1 mode : %s\t\n", enum_to_string(BMA5_INT1_MODE_LATCHED));
    // printf("Int1 output type : %s\t\n", enum_to_string(BMA5_INT1_OD_PUSH_PULL));
    // printf("Int1 level : %s\t\n", enum_to_string(BMA5_INT1_LVL_ACTIVE_HIGH));

    // printf("\nCount, Accel_LSB_X, Accel_LSB_Y, Accel_LSB_Z, Acc_ms2_X, Acc_ms2_Y, Acc_ms2_Z\n");

}

void bma530_readAccel(void)
{
    /* Get accel data ready status */
    rslt = bma5_get_sensor_status(&status, &dev);
    // bma5_check_rslt("bma5_get_sensor_status", rslt);

    if (status.acc_data_rdy)
    {
        /* Get accel data ready interrupt status */
        // rslt = bma530_get_int_status(&int_status, n_status, &dev);
        // bma5_check_rslt("bma530_get_int_status", rslt);

        /* Check if the data is ready */
        // if (int_status.int_status.acc_drdy_int_status & BMA530_ACC_DRDY_INT_STATUS_MSK)
        {
            // rslt = bma5_set_sensor_status(&status, &dev);
            // bma5_check_rslt("bma5_set_sensor_status", rslt);

            // rslt = bma530_set_int_status(&int_status, n_status, &dev);
            // bma5_check_rslt("bma530_set_int_status_int1_0", rslt);

            /* Get accel data and sensortime */
            rslt = bma5_get_acc(&sens_data, &dev);

            rslt = bma5_get_acc(&sens_data, &dev);
            // bma5_check_rslt("bma5_get_acc", rslt);

            /* Converting lsb to meter per second squared for 16 bit resolution at 2G range */
            cami_accel[0] = lsb_to_ms2(sens_data.x, (float)2, BMA5_16_BIT_RESOLUTION);
            cami_accel[1] = lsb_to_ms2(sens_data.y, (float)2, BMA5_16_BIT_RESOLUTION);
            cami_accel[2] = lsb_to_ms2(sens_data.z, (float)2, BMA5_16_BIT_RESOLUTION);

            /* Print the data in m/s2 */
            // printf("%4d  %11d  %11d  %11d  %9.2f  %9.2f  %9.2f\n",
            //         loop,
            //         sens_data.x,
            //         sens_data.y,
            //         sens_data.z,
            //         x,
            //         y,
            //         z);

            loop++;
        }
    }
    
    // bma5_coines_deinit();

}

/*!
 *  @brief This internal API converts raw sensor values(LSB) to meters per seconds square.
 */
static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}
