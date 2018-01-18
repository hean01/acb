/*
 * Arcade controller board LKM
 *
 * Copyright 2017 Henrik Andersson <henrik.4e@gmail.com>
 *
 * Arcade controller board LKM is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * Arcade controller board LKM is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Arcade controller board LKM.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>

#define MAX_DEVICES 7

/* mcp23017 registers */
#define IODIRA		0x00
#define IODIRB		0x01
#define IPOLA		0x02
#define IPOLB		0x03
#define GPINTENA	0x04
#define GPINTENB	0x05
#define DEFVALA		0x06
#define DEFVALB		0x07
#define INTCONA		0x08
#define INTCONB		0x09
#define IOCON		0x0a
#define GPPUA		0x0c
#define GPPUB		0x0d
#define INTFA		0x0e
#define INTFB		0x0f
#define INTCAPA		0x10
#define INTCAPB		0x11
#define GPIOA		0x12
#define GPIOB		0x13
#define OLATA		0x14
#define OLATB		0x15


struct i2c_client *clients[MAX_DEVICES];

unsigned int i2c_bus = 1;

/* defaults for Raspberry Pi 3 and clones */
char *boards[MAX_DEVICES] = {
  "player:0x20:17",
  "player:0x21:27",
  "auxiliary:0x22:22",
  NULL,
  NULL,
  NULL,
  NULL
};
int board_cnt = 3;

module_param(i2c_bus, uint, 0);
MODULE_PARM_DESC(i2c_bus, "Specify which i2c bus were ACB's are connected (default 1)");

module_param_array(boards, charp, &board_cnt, 0);
MODULE_PARM_DESC(boards, "Array of board definitons \"<type>:<addr>:<gpio>\", ex. \"player:0x20:17\"");

unsigned int acb_gpio[MAX_DEVICES];
struct i2c_board_info acb_i2c_boards[MAX_DEVICES];


typedef enum acb_driver_type {
  PlayerController = 0,
  AuxiliaryController,
} acb_driver_type;

struct acb_device {
  int instance;
  struct i2c_client *client;
  const struct i2c_device_id *id;
  unsigned char gpio;
  struct input_dev *input;
  uint16_t port;
};

short auxiliary_input_to_key_map[] = {
  BTN_0,
  BTN_1,
  BTN_2,
  BTN_3,
  BTN_4,
  BTN_5,
  BTN_6,
  BTN_7
};

short player_input_to_key_map[] = {
  0,0,0,0, /* abs joystick input */
  BTN_A,
  BTN_B,
  BTN_C,
  BTN_X,
  BTN_Y,
  BTN_Z,
  BTN_TL,
  BTN_TR,
  BTN_MISC,
  BTN_MODE,
  BTN_START,
  BTN_SELECT,
};

static irq_handler_t
acb_threaded_irq_handler(int irq, void *opaque)
{
  int i;
  unsigned char a,b;
  struct acb_device *dev;
  dev = (struct acb_device *)opaque;

  /* read INTCAPA/B and update gpio data, only read porta on auxiliary
     boards, this read clears the interrupt */
  a = i2c_smbus_read_byte_data(dev->client, INTCAPA);
  b = i2c_smbus_read_byte_data(dev->client, INTCAPB);

  dev->port = ((unsigned short)a << 8) & 0xff00;
  if (dev->id->driver_data != AuxiliaryController)
    dev->port |= b & 0xff;

  switch (dev->id->driver_data)
  {
  case PlayerController:
    // printk(KERN_DEBUG "%s: Player controller interrupt input: %x\n", __func__, dev->port);

    /* process joystick, portb gpio 0 - 3 */
    input_report_abs(dev->input, ABS_X, ((dev->port >> 0) & 1) - ((dev->port >> 1) & 1));
    input_report_abs(dev->input, ABS_Y, ((dev->port >> 2) & 1) - ((dev->port >> 3) & 1));

    /* process buttons */
    for (i = 4; i < 16; i++)
    {
      input_report_key(dev->input, player_input_to_key_map[i], !((dev->port >> i)&1));
    }

    break;

  case AuxiliaryController:
    for (i = 0; i < 8; i++)
    {
      input_report_key(dev->input, auxiliary_input_to_key_map[i], !((dev->port >> i) & 1));
    }
    break;
  }

  input_sync(dev->input);

  return (irq_handler_t)IRQ_HANDLED;
}

/* Initialize evdev input device for the specified arcade controller
   board and register it with the kernel */
static int acb_input_device_init(struct acb_device *dev)
{
  int i, ret;

  /* setup input device dependent on type */
  dev->input = input_allocate_device();
  if (dev->input == NULL)
  {
    printk(KERN_ERR "%s: not enough memory\n", __func__);
    kfree(dev);
    return -ENOMEM;
  }

  switch(dev->id->driver_data)
  {
  case PlayerController:
    dev->input->name = "Arcade player controller";
    dev->input->evbit[0] =  BIT_MASK(EV_KEY) |  BIT_MASK(EV_ABS);
    input_set_abs_params(dev->input, ABS_X, -1, 1, 0, 0);
    input_set_abs_params(dev->input, ABS_Y, -1, 1, 0, 0);

    for (i = 4; i < 16; i++)
      set_bit(player_input_to_key_map[i], dev->input->keybit);

    break;

  case AuxiliaryController:
    dev->input->name = "Arcade auxiliary controller";
    dev->input->evbit[0] =  BIT_MASK(EV_KEY);
    for (i = 0; i < 8; i++)
      set_bit(auxiliary_input_to_key_map[i], dev->input->keybit);
    break;
  }

  /* register the input device */
  ret = input_register_device(dev->input);
  if (ret != 0)
  {
    printk(KERN_ERR "%s: failed to register input device\n", __func__);
    input_free_device(dev->input);
    kfree(dev);
    return ret;
  }

  return 0;
}

static void acb_input_device_deinit(struct acb_device *dev)
{
  input_unregister_device(dev->input);
  dev->input = NULL;
}

/* Setup the mcs23017 i2c device as a arcade controller board */
static int acb_mcp23017_init(struct acb_device *dev)
{
  i2c_smbus_write_byte_data(dev->client, IODIRA, 0xff);
  i2c_smbus_write_byte_data(dev->client, IODIRB, 0xff);
  i2c_smbus_write_byte_data(dev->client, IPOLA, 0x00);
  i2c_smbus_write_byte_data(dev->client, IPOLB, 0x00);
  i2c_smbus_write_byte_data(dev->client, GPINTENA, 0xff);
  i2c_smbus_write_byte_data(dev->client, GPINTENB, 0xff);
  i2c_smbus_write_byte_data(dev->client, DEFVALA, 0x00);
  i2c_smbus_write_byte_data(dev->client, DEFVALB, 0x00);
  i2c_smbus_write_byte_data(dev->client, INTCONA, 0x00);
  i2c_smbus_write_byte_data(dev->client, INTCONB, 0x00);
  i2c_smbus_write_byte_data(dev->client, IOCON, 0b01000010);
  i2c_smbus_write_byte_data(dev->client, GPPUA, 0xff);
  i2c_smbus_write_byte_data(dev->client, GPPUB, 0xff);

  if (dev->id->driver_data == AuxiliaryController)
  {
    /* set port b as outputs */
    i2c_smbus_write_byte_data(dev->client, IODIRB, 0x00);
    i2c_smbus_write_byte_data(dev->client, GPINTENB, 0x00);
    i2c_smbus_write_byte_data(dev->client, GPPUB, 0x00);
  }

  /* read intcap to clear any interrupts */
  i2c_smbus_read_byte_data(dev->client, INTCAPA);
  i2c_smbus_read_byte_data(dev->client, INTCAPB);
  return 0;
}

static int acb_device_setup_gpio_irq(struct acb_device *dev)
{
  int ret;
  char name[32];
  static uint8_t cnt = 1;

  if (!gpio_is_valid(dev->gpio))
  {
    printk(KERN_ERR "%s: Invalid gpio %d\n", __func__, dev->gpio);
    return -EINVAL;
  }

  /* request and setup gpio for interrupt */
  snprintf(name, sizeof(name), "acb_interrupt_%d", cnt);
  ret = gpio_request_one(dev->gpio, GPIOF_IN, name);
  if (ret < 0)
  {
    printk(KERN_ERR "%s: Unable to request gpio %d (%d)\n", __func__, dev->gpio, ret);
    return ret;
  }

  // gpio_set_debounce(dev->gpio, 10);

  /* setup IRQ */
  ret = gpio_to_irq(dev->gpio);
  if (ret < 0)
  {
    gpio_free(dev->gpio);
    printk(KERN_ERR "%s: Unable to get IRQ for gpio %d (%d)\n", __func__, dev->gpio, ret);
    return ret;
  }

  dev->client->irq = ret;

  // TODO: Do we need individual ISR for each device instance ?
  ret = request_threaded_irq(dev->client->irq,
			     NULL, (irq_handler_t) acb_threaded_irq_handler,
			     IRQF_TRIGGER_HIGH|IRQF_ONESHOT,
			     "acb_interrupt_handler", dev);
  if (ret != 0)
  {
    gpio_free(dev->gpio);
    printk(KERN_ERR "%s: failed to request IRQ %d (%d)\n", __func__, dev->client->irq, ret);
    return -EIO;
  }

  // printk(KERN_DEBUG "%s: gpio %d interrupt mapped to IRQ %d\n", __func__, dev->gpio, dev->client->irq);
  cnt++;
  return 0;
}

static int acb_device_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
  int ret;
  struct acb_device *dev;
  /*
   * Simple detect of expected i2c chip on specified address
   *
   * The tests work against POR values eg. power on reset and only works once.
   *
   * TODO: maybe we should add a reset gpio which cuts power to arcade boards to
   * ensure that the chips are in POR state...
   *
   * FIXME: This detection fails if one have loaded an aux on the
   * address without a POR, eg the IODIR registers are not as
   * expected.
   */

  /* Read IODIRA and IODIRB values which is all set to ones upon reset */
  if (i2c_smbus_read_byte_data(client, IODIRA) != 0xff)
    goto detect_chip_failed;
  if (i2c_smbus_read_byte_data(client, IODIRB) != 0xff)
    goto detect_chip_failed;

  /* Read IPOLA and IPOLB values which is all set to zeros upon reset */
  if (i2c_smbus_read_byte_data(client, IPOLA) != 0x00)
    goto detect_chip_failed;
  if (i2c_smbus_read_byte_data(client, IPOLB) != 0x00)
    goto detect_chip_failed;

  /*
   * Detection tests passed, lets assume from now that we have an
   * MCP23017 chip on the specified address.
   */

  dev = (struct acb_device *) client->dev.platform_data;
  i2c_set_clientdata(client, dev);
  dev->client = client;
  dev->id = id;

  /* initialize i2c device and setup GPIO on mcp23017 */
  acb_mcp23017_init(dev);

  /* initialize input device for arcade controller board */
  ret = acb_input_device_init(dev);
  if (ret != 0)
    goto free_acb_device;

  /* initialize gpio and IRQ */
  if (acb_device_setup_gpio_irq(dev) != 0)
    goto free_acb_input_device;

  printk(KERN_INFO "%s: %s device @ 0x%x using IRQ %d initialized and is ready to use\n", __func__,
	 (id->driver_data == 0 ? "Player" : "Auxilliary"),
	 client->addr, client->irq);

  return 0;

 free_acb_input_device:
  acb_input_device_deinit(dev);

 free_acb_device:
  kfree(dev);

  return ret;

 detect_chip_failed:
  printk(KERN_ERR "%s: device @ 0x%x is not an arcade controller board\n", __func__, client->addr);
  return -ENODEV;
}

static int acb_device_remove(struct i2c_client *client)
{
  struct acb_device *dev;

  dev = i2c_get_clientdata(client);
  if (dev == NULL)
    return -ENODEV;

  printk(KERN_INFO "%s: Remove of device @ 0x%x using IRQ %d\n", __func__,
	 dev->client->addr, dev->client->irq);

  acb_input_device_deinit(dev);

  free_irq(dev->client->irq, dev);
  gpio_free(dev->gpio);

  kfree(dev);
  return 0;
}

static const struct i2c_device_id acb_device_id[] = {
  {"acb_player", PlayerController},
  {"acb_auxiliary", AuxiliaryController},
  {}
};

static struct i2c_driver acb_device_driver = {
  .driver = {
    .name = "acb",
    .owner = THIS_MODULE,
  },

  .id_table = acb_device_id,
  .probe = acb_device_probe,
  .remove = acb_device_remove,
};

static int acb_build_boards(void)
{
  int i, ret, skip;

  skip = 0;
  for (i = 0; i < board_cnt; i++)
  {
    ret = sscanf(boards[i], "player:0x%02hx:%d", &acb_i2c_boards[i].addr, &acb_gpio[i]);
    if (ret == 2)
    {
      snprintf(acb_i2c_boards[i].type, sizeof(acb_i2c_boards[i].type), "%s", "acb_player");
      continue;
    }


    ret = sscanf(boards[i], "auxiliary:0x%02hx:%d", &acb_i2c_boards[i].addr, &acb_gpio[i]);
    if (ret == 2)
    {
      snprintf(acb_i2c_boards[i].type, sizeof(acb_i2c_boards[i].type), "%s", "acb_auxiliary");
      continue;
    }


    printk(KERN_ERR "%s: Skipping invalid board description '%s'", __func__, boards[i]);
    skip++;
  }

  board_cnt -= skip;

  return 0;
}

static int __init acb_init(void)
{
  int i, cnt, ret;
  struct i2c_adapter *adapter;

  printk(KERN_NOTICE "%s: Initialization of arcade controller boards on I2C bus %d", __func__, i2c_bus);

  memset(acb_i2c_boards, 0, sizeof(struct i2c_board_info) * MAX_DEVICES);
  /* build up acb_boards from boards */
  ret = acb_build_boards();
  if (ret != 0)
    return -EINVAL;

  if (board_cnt == 0)
  {
    printk(KERN_ERR "%s: No board descriptions, exiting...", __func__);
    return -EINVAL;
  }

  /* get adapter for specified i2c bus */
  adapter = i2c_get_adapter(i2c_bus);
  if (adapter == NULL)
  {
    printk(KERN_ERR "%s: Invalid I2C bus number %d, use module param i2c_bus to specify one\n", __func__, i2c_bus);
    return -EINVAL;
  }

  /* register i2c driver with kernel */
  ret = i2c_add_driver(&acb_device_driver);
  if (ret != 0)
  {
    printk(KERN_ERR "%s: Failed to register driver (%d)", __func__, ret);
    i2c_del_driver(&acb_device_driver);
    return ret;
  }

  /* create instances */
  for (i = 0; i < board_cnt; i++)
  {
    struct acb_device *dev;

    /* allocate and initialize acb_device */
    dev = kzalloc(sizeof(struct acb_device), GFP_KERNEL);
    if (dev == NULL)
    {
      printk(KERN_ERR "%s: not enough memory\n", __func__);
      ret = -ENOMEM;
    }
    dev->instance = i;
    dev->gpio = acb_gpio[i];

    printk(KERN_NOTICE "%s: Creating %s instance @ 0x%x using gpio %d", __func__,
	   acb_i2c_boards[i].type, acb_i2c_boards[i].addr, acb_gpio[i]);

    acb_i2c_boards[i].platform_data = dev;

    clients[i] = i2c_new_device(adapter, &acb_i2c_boards[i]);
  }

  /* verify that we succefully detected and created at least one instance */
  cnt = 0;
  for (i = 0; i < MAX_DEVICES; i++)
  {
    if (clients[i])
      cnt++;
  }

  if (cnt == 0)
  {
    printk(KERN_ERR "%s: No Arcade controller boards detected, did you specify correct i2c bus?.", __func__);
    i2c_del_driver(&acb_device_driver);
    return -ENODEV;
  }

  return 0;
}

static void __exit acb_exit(void)
{
  int i;

  for (i = 0; i < MAX_DEVICES; i++)
  {
    if (!clients[i])
      continue;

    i2c_unregister_device(clients[i]);
  }

  i2c_del_driver(&acb_device_driver);
}

module_init(acb_init);
module_exit(acb_exit);

MODULE_AUTHOR("Henrik Andersson");
MODULE_DESCRIPTION("Arcade controller board driver");
MODULE_ALIAS("acb_player");
MODULE_ALIAS("acb_auxiliary");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(i2c, acb_device_id);

