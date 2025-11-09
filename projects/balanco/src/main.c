#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <inttypes.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

// ======== CONFIGURAÇÃO DO ADC ========

#if !DT_NODE_HAS_STATUS(DT_PATH(zephyr_user), okay)
#error "No suitable devicetree node found for ADC."
#endif

#define ADC_CHANNEL_NODE DT_PATH(zephyr_user)
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET_BY_IDX(ADC_CHANNEL_NODE, 0);

// Limites do ADC (0-4095) e limites reais do sensor (130-2900)
#define ADC_INPUT_MIN 130
#define ADC_INPUT_MAX 2370
#define ADC_INPUT_RANGE (ADC_INPUT_MAX - ADC_INPUT_MIN)

#define REMAP_MAX 6000
#define REMAP_MIN 3000
#define REMAP_SPN (REMAP_MAX - REMAP_MIN)

static int16_t sample_buffer;
static int32_t remap = REMAP_MIN;

void adc_read_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    if (!device_is_ready(adc_channel.dev)) {
        LOG_ERR("ADC device not ready");
        return;
    }

    struct adc_channel_cfg cfg = {
        .gain = ADC_GAIN_1_4,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id = adc_channel.channel_id,
        .differential = 0,
    };

    if (adc_channel_setup(adc_channel.dev, &cfg) < 0) {
        LOG_ERR("ADC setup failed");
        return;
    }

    const struct adc_sequence sequence = {
		.options     = NULL,
		.channels    = BIT(adc_channel.channel_id),
		.buffer      = &sample_buffer,
		.buffer_size = sizeof(sample_buffer),
		.resolution  = 12,
	};
    
    while (1) {
        int err = adc_read(adc_channel.dev, &sequence);
        if (err) LOG_ERR("ADC read error: %d", err);
        else {
            int32_t clamped_value = sample_buffer;

            // 1. Limita (clampa) o valor lido ao intervalo útil definido
            if (clamped_value < ADC_INPUT_MIN) {
                clamped_value = ADC_INPUT_MIN;
            }
            if (clamped_value > ADC_INPUT_MAX) {
                clamped_value = ADC_INPUT_MAX;
            }

            // 2. Remapeia o valor limitado (ex: 130-2900) para o percentual (1-100%)
            int32_t percentage = ((clamped_value - ADC_INPUT_MIN) * 100) / ADC_INPUT_RANGE;
            remap = REMAP_MIN + (percentage * REMAP_SPN) / 100;
            LOG_INF("Leitura ADC: %d (bruto) -> %d%%, valor remapeado: %d", sample_buffer, percentage, remap);
        }
        k_msleep(100);
    }
}




// ======== CONFIGURAÇÃO DO MOTOR ========

#define APLUS   DT_ALIAS(coilaplus)
#define AMINUS  DT_ALIAS(coilaminus)
#define BPLUS   DT_ALIAS(coilbplus)
#define BMINUS  DT_ALIAS(coilbminus)

static const struct gpio_dt_spec coil_a_plus  = GPIO_DT_SPEC_GET(APLUS, gpios);
static const struct gpio_dt_spec coil_a_minus = GPIO_DT_SPEC_GET(AMINUS, gpios);
static const struct gpio_dt_spec coil_b_plus  = GPIO_DT_SPEC_GET(BPLUS, gpios);
static const struct gpio_dt_spec coil_b_minus = GPIO_DT_SPEC_GET(BMINUS, gpios);

//#define STEP_DELAY_MS K_USEC(2000)

static inline void step_forward(void)
{
    gpio_pin_set_dt(&coil_a_plus, 1);
    gpio_pin_set_dt(&coil_a_minus, 0);
    gpio_pin_set_dt(&coil_b_plus, 1);
    gpio_pin_set_dt(&coil_b_minus, 0);
    k_sleep(K_USEC(remap));

    gpio_pin_set_dt(&coil_a_plus, 0);
    gpio_pin_set_dt(&coil_a_minus, 1);
    gpio_pin_set_dt(&coil_b_plus, 1);
    gpio_pin_set_dt(&coil_b_minus, 0);
    k_sleep(K_USEC(remap));

    gpio_pin_set_dt(&coil_a_plus, 0);
    gpio_pin_set_dt(&coil_a_minus, 1);
    gpio_pin_set_dt(&coil_b_plus, 0);
    gpio_pin_set_dt(&coil_b_minus, 1);
    k_sleep(K_USEC(remap));

    gpio_pin_set_dt(&coil_a_plus, 1);
    gpio_pin_set_dt(&coil_a_minus, 0);
    gpio_pin_set_dt(&coil_b_plus, 0);
    gpio_pin_set_dt(&coil_b_minus, 1);
    k_sleep(K_USEC(remap));
}

static inline void step_backward(void)
{
    gpio_pin_set_dt(&coil_a_plus, 1);
    gpio_pin_set_dt(&coil_a_minus, 0);
    gpio_pin_set_dt(&coil_b_plus, 0);
    gpio_pin_set_dt(&coil_b_minus, 1);
    k_sleep(K_USEC(remap));

    gpio_pin_set_dt(&coil_a_plus, 0);
    gpio_pin_set_dt(&coil_a_minus, 1);
    gpio_pin_set_dt(&coil_b_plus, 0);
    gpio_pin_set_dt(&coil_b_minus, 1);
    k_sleep(K_USEC(remap));

    gpio_pin_set_dt(&coil_a_plus, 0);
    gpio_pin_set_dt(&coil_a_minus, 1);
    gpio_pin_set_dt(&coil_b_plus, 1);
    gpio_pin_set_dt(&coil_b_minus, 0);
    k_sleep(K_USEC(remap));

    gpio_pin_set_dt(&coil_a_plus, 1);
    gpio_pin_set_dt(&coil_a_minus, 0);
    gpio_pin_set_dt(&coil_b_plus, 1);
    gpio_pin_set_dt(&coil_b_minus, 0);
    k_sleep(K_USEC(remap));
}

static inline void step_off(void)
{
    gpio_pin_set_dt(&coil_a_plus, 0);
    gpio_pin_set_dt(&coil_a_minus, 0);
    gpio_pin_set_dt(&coil_b_plus, 0);
    gpio_pin_set_dt(&coil_b_minus, 0);
}

void motor_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        //printk("Girando no sentido horário...\n");
        //for (int i = 0; i < 1000; i++) step_forward();
        //step_off();
        //k_sleep(K_SECONDS(2));

        printk("Girando no sentido anti-horário...\n");
        //for (int i = 0; i < 1000; i++)
        step_backward();
        //step_off();
        //k_sleep(K_SECONDS(2));
    }
}

// ======== MAIN ========

#define MOTOR_STACK 16384
#define ADC_STACK   4096

K_THREAD_STACK_DEFINE(motor_stack, MOTOR_STACK);
K_THREAD_STACK_DEFINE(adc_stack, ADC_STACK);

struct k_thread motor_data;
struct k_thread adc_data;

int main(void)
{
    printk("Teste de Motor de Passo Bipolar + ADC com Zephyr\n");

    const struct gpio_dt_spec *coils[] = {
        &coil_a_plus, &coil_a_minus, &coil_b_plus, &coil_b_minus
    };

    for (int i = 0; i < ARRAY_SIZE(coils); i++) {
        if (!device_is_ready(coils[i]->port)) {
            printk("Erro: GPIO %s não está pronto!\n", coils[i]->port->name);
            return 1;
        }
        int ret = gpio_pin_configure_dt(coils[i], GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            printk("Erro %d ao configurar pino %d\n", ret, coils[i]->pin);
            return 1;
        }
    }

    k_thread_create(&motor_data, motor_stack, K_THREAD_STACK_SIZEOF(motor_stack),
                    motor_thread, NULL, NULL, NULL, 5, 0, K_NO_WAIT);

    k_thread_create(&adc_data, adc_stack, K_THREAD_STACK_SIZEOF(adc_stack),
                    adc_read_thread, NULL, NULL, NULL, 7, 0, K_MSEC(100));
    
    return 0;
}
