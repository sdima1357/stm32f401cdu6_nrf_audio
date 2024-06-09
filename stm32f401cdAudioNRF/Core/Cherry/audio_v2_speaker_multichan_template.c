#include "usbd_core.h"
#include "usbd_audio.h"
#include "usbd_hid.h"

#define USBD_VID           0xffff
#define USBD_PID           0xffff
#define USBD_MAX_POWER     500
#define USBD_LANGID_STRING 1033

#ifdef CONFIG_USB_HS
#define EP_INTERVAL 0x04
#else
#define EP_INTERVAL 0x01
#endif

#define HID_INT_EP          0x82
#define HID_INT_EP_SIZE     8
#define HID_INT_EP_INTERVAL 10

#define USB_HID_CONFIG_DESC_SIZ       (28+1)
static const uint8_t hid_keyboard_report_desc2[] = {
    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x06, // USAGE (Keyboard)
    0xa1, 0x01, // COLLECTION (Application)
	0x85, 0x01, /* Report ID1  */
    0x05, 0x07, // USAGE_PAGE (Keyboard)
    0x19, 0xe0, // USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7, // USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00, // LOGICAL_MINIMUM (0)
    0x25, 0x01, // LOGICAL_MAXIMUM (1)
    0x75, 0x01, // REPORT_SIZE (1)
    0x95, 0x08, // REPORT_COUNT (8)
    0x81, 0x02, // INPUT (Data,Var,Abs)
    0x95, 0x01, // REPORT_COUNT (1)
    0x75, 0x08, // REPORT_SIZE (8)
    0x81, 0x03, // INPUT (Cnst,Var,Abs)
    0x95, 0x05, // REPORT_COUNT (5)
    0x75, 0x01, // REPORT_SIZE (1)
    0x05, 0x08, // USAGE_PAGE (LEDs)
    0x19, 0x01, // USAGE_MINIMUM (Num Lock)
    0x29, 0x05, // USAGE_MAXIMUM (Kana)
    0x91, 0x02, // OUTPUT (Data,Var,Abs)
    0x95, 0x01, // REPORT_COUNT (1)
    0x75, 0x03, // REPORT_SIZE (3)
    0x91, 0x03, // OUTPUT (Cnst,Var,Abs)
    0x95, 0x06, // REPORT_COUNT (6)
    0x75, 0x08, // REPORT_SIZE (8)
    0x15, 0x00, // LOGICAL_MINIMUM (0)
    0x25, 0xFF, // LOGICAL_MAXIMUM (255)
    0x05, 0x07, // USAGE_PAGE (Keyboard)
    0x19, 0x00, // USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65, // USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00, // INPUT (Data,Ary,Abs)
    0xc0 ,       // END_COLLECTION
    //MEDIA KEYBOARD
    0x05, 0x0C,    /*      Usage Page (Consumer Devices)      */
    0x09, 0x01,    /*      Usage (Consumer Control)         */
    0xA1, 0x01,    /*      Collection (Application)         */
    0x85, 0x02,    /* Report ID2  */
    0x05, 0x0C,    /*      Usage Page (Consumer Devices)      */
    0x15, 0x00,    /*      Logical Minimum (0)               */
    0x25, 0x01,    /*      Logical Maximum (1)               */
    0x75, 0x01,    /*      Report Size (1)                  */
    0x95, 0x07,    /*      Report Count (7)               */
    0x09, 0xB5,    /*      Usage (Scan Next Track)            */
    0x09, 0xB6,    /*      Usage (Scan Previous Track)         */
    0x09, 0xB7,    /*      Usage (Stop)                  */
    0x09, 0xCD,    /*      Usage (Play / Pause)            */
    0x09, 0xE2,    /*      Usage (Mute)                  */
    0x09, 0xE9,    /*      Usage (Volume Up)               */
    0x09, 0xEA,    /*      Usage (Volume Down)               */
    0x81, 0x02,    /*      Input (Data, Variable, Absolute)   */
    0x95, 0x01,    /*      Report Count (1)               */
    0x81, 0x01,    /*      Input (Constant)               */
    0xC0          /*      End Collection                  */

};
static const uint8_t hid_keyboard_report_descS [] =
{
  //hid report descriptor for interface 1 (keyboard)
   0x05, 0x01, //usage page (generic desktop)   //52, 53
   0x09, 0x06, //usage (keyboard)   //54, 55
   0xA1, 0x01, //collection (application) //56, 57
   0x85, 0x01, /* Report ID1  */
   0x05, 0x07, //usage page (key codes)   //58, 59
   0x19, 0xE0, //usage min (224) //60, 61
   0x29, 0xE7, //usage max (231) //62, 63
   0x15, 0x00, //logical min (0) //64, 65
   0x25, 0x01, //logical max (1) //66, 67
   0x75, 0x01, //report size (1) //68, 69
   0x95, 0x08, //report count (8)   //70, 71
   0x81, 0x02, //input (data, variable, absolute) [modifier byte] //72, 73
   0x95, 0x01, //report count (1)   //74, 75
   0x75, 0x08, //report size (8)    //76, 77
   0x81, 0x01, //input (constant) [reserved byte]  //78, 79
   0x95, 0x05, //report count (5)   //80, 81
   0x75, 0x01, //report size (1)    //82, 83
   0x05, 0x08, //usage page (page# for leds) //84, 85
   0x19, 0x01, //usage min (1)   //86, 87
   0x29, 0x05, //usage max (5)   //88, 89
   0x91, 0x02, //output (data, var, abs) [led report] //90, 91
   0x95, 0x01, //report count (1)   //92, 93
   0x75, 0x03, //report size (3) //94, 95
   0x91, 0x01, //output (constant) [led report padding]  //96, 97
   0x95, 0x05, //report count (5)   //98, 99
   0x75, 0x08, //report size (8) //100, 101
   0x15, 0x00, //logical min (0) //102, 103
   0x25, 0x65, //logical max (101)  //104, 105
   0x05, 0x07, //usage page (key codes)   //106, 107
   0x19, 0x00, //usage min (0)   //108, 109
   0x29, 0x65, //usage max (101) //110, 111
   0x81, 0x00, //input (data, array)   //112, 113
   0xC0,        //end collection  //114

   //MEDIA KEYBOARD
   0x05, 0x0C,    /*      Usage Page (Consumer Devices)      */
   0x09, 0x01,    /*      Usage (Consumer Control)         */
   0xA1, 0x01,    /*      Collection (Application)         */
   0x85, 0x02,    /* Report ID2  */
   0x05, 0x0C,    /*      Usage Page (Consumer Devices)      */
   0x15, 0x00,    /*      Logical Minimum (0)               */
   0x25, 0x01,    /*      Logical Maximum (1)               */
   0x75, 0x01,    /*      Report Size (1)                  */
   0x95, 0x07,    /*      Report Count (7)               */
   0x09, 0xB5,    /*      Usage (Scan Next Track)            */
   0x09, 0xB6,    /*      Usage (Scan Previous Track)         */
   0x09, 0xB7,    /*      Usage (Stop)                  */
   0x09, 0xCD,    /*      Usage (Play / Pause)            */
   0x09, 0xE2,    /*      Usage (Mute)                  */
   0x09, 0xE9,    /*      Usage (Volume Up)               */
   0x09, 0xEA,    /*      Usage (Volume Down)               */
   0x81, 0x02,    /*      Input (Data, Variable, Absolute)   */
   0x95, 0x01,    /*      Report Count (1)               */
   0x81, 0x01,    /*      Input (Constant)               */
   0xC0          /*      End Collection                  */
};
static const uint8_t hid_keyboard_report_desc [] =
{
  //hid report descriptor for interface 1 (keyboard)
   0x05, 0x01, //usage page (generic desktop)   //52, 53
   0x09, 0x06, //usage (keyboard)   //54, 55
   0xA1, 0x01, //collection (application) //56, 57
   0x85, 0x01, /* Report ID1  */
   0x05, 0x07, //usage page (key codes)   //58, 59
   0x19, 0xE0, //usage min (224) //60, 61
   0x29, 0xE7, //usage max (231) //62, 63
   0x15, 0x00, //logical min (0) //64, 65
   0x25, 0x01, //logical max (1) //66, 67
   0x75, 0x01, //report size (1) //68, 69
   0x95, 0x08, //report count (8)   //70, 71
   0x81, 0x02, //input (data, variable, absolute) [modifier byte] //72, 73
   0x95, 0x01, //report count (1)   //74, 75
   0x75, 0x08, //report size (8)    //76, 77
   0x81, 0x01, //input (constant) [reserved byte]  //78, 79
   0x95, 0x05, //report count (5)   //80, 81
   0x75, 0x01, //report size (1)    //82, 83
   0x05, 0x08, //usage page (page# for leds) //84, 85
   0x19, 0x01, //usage min (1)   //86, 87
   0x29, 0x05, //usage max (5)   //88, 89
   0x91, 0x02, //output (data, var, abs) [led report] //90, 91
   0x95, 0x01, //report count (1)   //92, 93
   0x75, 0x03, //report size (3) //94, 95
   0x91, 0x01, //output (constant) [led report padding]  //96, 97
   0x95, 0x05, //report count (5)   //98, 99
   0x75, 0x08, //report size (8) //100, 101
   0x15, 0x00, //logical min (0) //102, 103
   0x25, 0x65, //logical max (101)  //104, 105
   0x05, 0x07, //usage page (key codes)   //106, 107
   0x19, 0x00, //usage min (0)   //108, 109
   0x29, 0x65, //usage max (101) //110, 111
   0x81, 0x00, //input (data, array)   //112, 113
   0xC0,        //end collection  //114

   0x05 ,0x0C,        //(GLOBAL) USAGE_PAGE         0x000C Consumer Device Page
   0x09 ,0x01,        //(LOCAL)  USAGE              0x000C0001 Consumer Control (Application Collection)
   0xA1 ,0x01,        //(MAIN)   COLLECTION         0x01 Application (Usage=0x000C0001: Page=Consumer Device Page, Usage=Consumer Control, Type=Application Collection)
   0x85 ,0x02,        //  (GLOBAL) REPORT_ID          0x01 (1)
   0x19 ,0x00,        //  (LOCAL)  USAGE_MINIMUM      0x000C0000 Unassigned  <-- Info: Consider replacing 19 00 with 18
   0x2A ,0x3C,0x02,   //     (LOCAL)  USAGE_MAXIMUM      0x000C023C AC Format (Selector)
   0x15 ,0x00,        //  (GLOBAL) LOGICAL_MINIMUM    0x00 (0)  <-- Info: Consider replacing 15 00 with 14
   0x26 ,0x3C,0x02,   //     (GLOBAL) LOGICAL_MAXIMUM    0x023C (572)
   0x95 ,0x01,        //  (GLOBAL) REPORT_COUNT       0x01 (1) Number of fields
   0x75 ,0x10,        //  (GLOBAL) REPORT_SIZE        0x10 (16) Number of bits per field
   0x81 ,0x00,        //  (MAIN)   INPUT              0x00000000 (1 field x 16 bits) 0=Data 0=Array 0=Absolute
   0xC0           //(MAIN)   END_COLLECTION     Application
#if 0
   //MEDIA KEYBOARD
   0x05, 0x0C,    /*      Usage Page (Consumer Devices)      */
   0x09, 0x01,    /*      Usage (Consumer Control)         */
   0xA1, 0x01,    /*      Collection (Application)         */
   0x85, 0x02,    /* Report ID2  */
   0x05, 0x0C,    /*      Usage Page (Consumer Devices)      */
   0x15, 0x00,    /*      Logical Minimum (0)               */
   0x25, 0x01,    /*      Logical Maximum (1)               */
   0x75, 0x01,    /*      Report Size (1)                  */
   0x95, 0x07,    /*      Report Count (7)               */
   0x09, 0xB5,    /*      Usage (Scan Next Track)            */
   0x09, 0xB6,    /*      Usage (Scan Previous Track)         */
   0x09, 0xB7,    /*      Usage (Stop)                  */
   0x09, 0xCD,    /*      Usage (Play / Pause)            */
   0x09, 0xE2,    /*      Usage (Mute)                  */
   0x09, 0xE9,    /*      Usage (Volume Up)               */
   0x09, 0xEA,    /*      Usage (Volume Down)               */
   0x81, 0x02,    /*      Input (Data, Variable, Absolute)   */
   0x95, 0x01,    /*      Report Count (1)               */
   0x81, 0x01,    /*      Input (Constant)               */
   0xC0          /*      End Collection                  */
#endif
};

static const uint8_t hid_keyboard_report_desc3[] = {
    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x06, // USAGE (Keyboard)
    0xa1, 0x01, // COLLECTION (Application)
    0x05, 0x07, // USAGE_PAGE (Keyboard)
    0x19, 0xe0, // USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7, // USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00, // LOGICAL_MINIMUM (0)
    0x25, 0x01, // LOGICAL_MAXIMUM (1)
    0x75, 0x01, // REPORT_SIZE (1)
    0x95, 0x08, // REPORT_COUNT (8)
    0x81, 0x02, // INPUT (Data,Var,Abs)
    0x95, 0x01, // REPORT_COUNT (1)
    0x75, 0x08, // REPORT_SIZE (8)
    0x81, 0x03, // INPUT (Cnst,Var,Abs)
    0x95, 0x05, // REPORT_COUNT (5)
    0x75, 0x01, // REPORT_SIZE (1)
    0x05, 0x08, // USAGE_PAGE (LEDs)
    0x19, 0x01, // USAGE_MINIMUM (Num Lock)
    0x29, 0x05, // USAGE_MAXIMUM (Kana)
    0x91, 0x02, // OUTPUT (Data,Var,Abs)
    0x95, 0x01, // REPORT_COUNT (1)
    0x75, 0x03, // REPORT_SIZE (3)
    0x91, 0x03, // OUTPUT (Cnst,Var,Abs)
    0x95, 0x06, // REPORT_COUNT (6)
    0x75, 0x08, // REPORT_SIZE (8)
    0x15, 0x00, // LOGICAL_MINIMUM (0)
    0x25, 0xFF, // LOGICAL_MAXIMUM (255)
    0x05, 0x07, // USAGE_PAGE (Keyboard)
    0x19, 0x00, // USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65, // USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00, // INPUT (Data,Ary,Abs)
    0xc0        // END_COLLECTION
};


#define HID_KEYBOARD_REPORT_DESC_SIZE (sizeof(hid_keyboard_report_desc))

#define AUDIO_OUT_EP 0x01

#define AUDIO_OUT_CLOCK_ID 0x01
#define AUDIO_OUT_FU_ID    0x03

#define AUDIO_FREQ      ((int)(USBD_AUDIO_FREQ))
#ifdef B24
#define HALF_WORD_BYTES 3  //2 half word (one channel)
#define SAMPLE_BITS     (HALF_WORD_BYTES*8) //16 bit per channel
#else
#define HALF_WORD_BYTES 2  //2 half word (one channel)
#define SAMPLE_BITS     (HALF_WORD_BYTES*8) //16 bit per channel
#endif

#define BMCONTROL (AUDIO_V2_FU_CONTROL_MUTE | AUDIO_V2_FU_CONTROL_VOLUME)

#define OUT_CHANNEL_NUM 2

#if OUT_CHANNEL_NUM == 1
#define OUTPUT_CTRL      DBVAL(BMCONTROL), DBVAL(BMCONTROL)
#define OUTPUT_CH_ENABLE 0x00000000
#elif OUT_CHANNEL_NUM == 2
#define OUTPUT_CTRL      DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL)
#define OUTPUT_CH_ENABLE 0x00000003
#elif OUT_CHANNEL_NUM == 3
#define OUTPUT_CTRL      DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL)
#define OUTPUT_CH_ENABLE 0x00000007
#elif OUT_CHANNEL_NUM == 4
#define OUTPUT_CTRL      DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL)
#define OUTPUT_CH_ENABLE 0x0000000f
#elif OUT_CHANNEL_NUM == 5
#define OUTPUT_CTRL      DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL)
#define OUTPUT_CH_ENABLE 0x0000001f
#elif OUT_CHANNEL_NUM == 6
#define OUTPUT_CTRL      DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL)
#define OUTPUT_CH_ENABLE 0x0000003F
#elif OUT_CHANNEL_NUM == 7
#define OUTPUT_CTRL      DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL)
#define OUTPUT_CH_ENABLE 0x0000007f
#elif OUT_CHANNEL_NUM == 8
#define OUTPUT_CTRL      DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL)
#define OUTPUT_CH_ENABLE 0x000000ff
#endif

#define AUDIO_OUT_PACKET ((((uint32_t)((AUDIO_FREQ * HALF_WORD_BYTES * OUT_CHANNEL_NUM+999) / 1000))+7)&~7)

#define USB_AUDIO_CONFIG_DESC_SIZ (9 +                                                     \
                                   AUDIO_V2_AC_DESCRIPTOR_INIT_LEN +                       \
                                   AUDIO_V2_SIZEOF_AC_CLOCK_SOURCE_DESC +                  \
                                   AUDIO_V2_SIZEOF_AC_INPUT_TERMINAL_DESC +                \
                                   AUDIO_V2_SIZEOF_AC_FEATURE_UNIT_DESC(OUT_CHANNEL_NUM) + \
                                   AUDIO_V2_SIZEOF_AC_OUTPUT_TERMINAL_DESC +               \
                                   AUDIO_V2_AS_DESCRIPTOR_INIT_LEN)+\
								   USB_HID_CONFIG_DESC_SIZ

#define AUDIO_AC_SIZ (AUDIO_V2_SIZEOF_AC_HEADER_DESC +                        \
                      AUDIO_V2_SIZEOF_AC_CLOCK_SOURCE_DESC +                  \
                      AUDIO_V2_SIZEOF_AC_INPUT_TERMINAL_DESC +                \
                      AUDIO_V2_SIZEOF_AC_FEATURE_UNIT_DESC(OUT_CHANNEL_NUM) + \
                      AUDIO_V2_SIZEOF_AC_OUTPUT_TERMINAL_DESC)

const uint8_t audio_v2_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_PID, 0x0001, 0x01),
    USB_CONFIG_DESCRIPTOR_INIT(USB_AUDIO_CONFIG_DESC_SIZ, 0x03, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    AUDIO_V2_AC_DESCRIPTOR_INIT(0x00, 0x02, AUDIO_AC_SIZ, AUDIO_CATEGORY_SPEAKER, 0x00, 0x00),
    AUDIO_V2_AC_CLOCK_SOURCE_DESCRIPTOR_INIT(AUDIO_OUT_CLOCK_ID, 0x03, 0x03),
    AUDIO_V2_AC_INPUT_TERMINAL_DESCRIPTOR_INIT(0x02, AUDIO_TERMINAL_STREAMING, 0x01, OUT_CHANNEL_NUM, OUTPUT_CH_ENABLE, 0x0000),
    AUDIO_V2_AC_FEATURE_UNIT_DESCRIPTOR_INIT(AUDIO_OUT_FU_ID, 0x02, OUTPUT_CTRL),
    AUDIO_V2_AC_OUTPUT_TERMINAL_DESCRIPTOR_INIT(0x04, AUDIO_OUTTERM_SPEAKER, 0x03, 0x01, 0x0000),
    AUDIO_V2_AS_DESCRIPTOR_INIT(0x01, 0x02, OUT_CHANNEL_NUM, OUTPUT_CH_ENABLE, HALF_WORD_BYTES, SAMPLE_BITS, AUDIO_OUT_EP, 0x09, AUDIO_OUT_PACKET, EP_INTERVAL),
    /************** Descriptor of Joystick Mouse interface ****************/
    /* 09 */
    0x09,                          /* bLength: Interface Descriptor size */
    USB_DESCRIPTOR_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type */
	0x02,                          /* bInterfaceNumber: Number of Interface */
    0x00,                          /* bAlternateSetting: Alternate setting */
    0x01,                          /* bNumEndpoints */
    0x03,                          /* bInterfaceClass: HID */
    0x01,                          /* bInterfaceSubClass : 1=BOOT, 0=no boot */
    0x01,                          /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
    0,                             /* iInterface: Index of string descriptor */
    /******************** Descriptor of Joystick Mouse HID ********************/
    /* 18 */
    0x09,                    /* bLength: HID Descriptor size */
    HID_DESCRIPTOR_TYPE_HID, /* bDescriptorType: HID */
    0x11,                    /* bcdHID: HID Class Spec release number */
    0x01,
    0x00,                          /* bCountryCode: Hardware target country */
    0x01,                          /* bNumDescriptors: Number of HID class descriptors to follow */
    0x22,                          /* bDescriptorType */
    HID_KEYBOARD_REPORT_DESC_SIZE, /* wItemLength: Total length of Report descriptor */
    0x00,
    /******************** Descriptor of Mouse endpoint ********************/
    /* 27 */
    0x07,                         /* bLength: Endpoint Descriptor size */
    USB_DESCRIPTOR_TYPE_ENDPOINT, /* bDescriptorType: */
    HID_INT_EP,                   /* bEndpointAddress: Endpoint Address (IN) */
    0x03,                         /* bmAttributes: Interrupt endpoint */
    HID_INT_EP_SIZE,              /* wMaxPacketSize: 4 Byte max */
    0x00,
    HID_INT_EP_INTERVAL, /* bInterval: Polling Interval */
    /* 34 */

    ///////////////////////////////////////
    /// string0 descriptor
    ///////////////////////////////////////
    USB_LANGID_INIT(USBD_LANGID_STRING),
    ///////////////////////////////////////
    /// string1 descriptor
    ///////////////////////////////////////
    0x14,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'C', 0x00,                  /* wcChar0 */
    'h', 0x00,                  /* wcChar1 */
    'e', 0x00,                  /* wcChar2 */
    'r', 0x00,                  /* wcChar3 */
    'r', 0x00,                  /* wcChar4 */
    'y', 0x00,                  /* wcChar5 */
    'U', 0x00,                  /* wcChar6 */
    'S', 0x00,                  /* wcChar7 */
    'B', 0x00,                  /* wcChar8 */
    ///////////////////////////////////////
    /// string2 descriptor
    ///////////////////////////////////////
    0x26,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'C', 0x00,                  /* wcChar0 */
    'h', 0x00,                  /* wcChar1 */
    'e', 0x00,                  /* wcChar2 */
    'r', 0x00,                  /* wcChar3 */
    'r', 0x00,                  /* wcChar4 */
    'y', 0x00,                  /* wcChar5 */
    'U', 0x00,                  /* wcChar6 */
    'S', 0x00,                  /* wcChar7 */
    'B', 0x00,                  /* wcChar8 */
    ' ', 0x00,                  /* wcChar9 */
    'U', 0x00,                  /* wcChar10 */
    'A', 0x00,                  /* wcChar11 */
    'C', 0x00,                  /* wcChar12 */
    ' ', 0x00,                  /* wcChar13 */
    'D', 0x00,                  /* wcChar14 */
    'E', 0x00,                  /* wcChar15 */
    'M', 0x00,                  /* wcChar16 */
    'O', 0x00,                  /* wcChar17 */
    ///////////////////////////////////////
    /// string3 descriptor
    ///////////////////////////////////////
    0x16,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    '2', 0x00,                  /* wcChar0 */
    '0', 0x00,                  /* wcChar1 */
    '2', 0x00,                  /* wcChar2 */
    '1', 0x00,                  /* wcChar3 */
    '0', 0x00,                  /* wcChar4 */
    '3', 0x00,                  /* wcChar5 */
    '1', 0x00,                  /* wcChar6 */
    '0', 0x00,                  /* wcChar7 */
    '0', 0x00,                  /* wcChar8 */
    '3', 0x00,                  /* wcChar9 */
#ifdef CONFIG_USB_HS
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x01,
    0x00,
#endif
    0x00
};


/* USB HID device Configuration Descriptor */
static uint8_t hid_desc[9] __ALIGN_END = {
    /* 18 */
    0x09,                    /* bLength: HID Descriptor size */
    HID_DESCRIPTOR_TYPE_HID, /* bDescriptorType: HID */
    0x11,                    /* bcdHID: HID Class Spec release number */
    0x01,
    0x00,                          /* bCountryCode: Hardware target country */
    0x01,                          /* bNumDescriptors: Number of HID class descriptors to follow */
    0x22,                          /* bDescriptorType */
    HID_KEYBOARD_REPORT_DESC_SIZE, /* wItemLength: Total length of Report descriptor */
    0x00,
};
/*
*/


static const uint8_t default_sampling_freq_table[] = {
    AUDIO_SAMPLE_FREQ_NUM(1),
    AUDIO_SAMPLE_FREQ_4B(AUDIO_FREQ),
    AUDIO_SAMPLE_FREQ_4B(AUDIO_FREQ),
    AUDIO_SAMPLE_FREQ_4B(0x00),
};

//USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t read_buffer[AUDIO_OUT_PACKET];


#define NP 2
#define ABSIZE (AUDIO_OUT_PACKET*NP)

//USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t read_buffer[ABSIZE];

extern void AUDIO_OUT_Start(void* pBuffer, uint32_t Size,int sampleBits);
void AUDIO_OUT_Periodic(void* pBuffer,int nbytes);
volatile int bpos = 0;
volatile bool rx_flag = 0;

void usbd_event_handler(uint8_t event)
{
    switch (event) {
        case USBD_EVENT_RESET:
            break;
        case USBD_EVENT_CONNECTED:
            break;
        case USBD_EVENT_DISCONNECTED:
            break;
        case USBD_EVENT_RESUME:
            break;
        case USBD_EVENT_SUSPEND:
            break;
        case USBD_EVENT_CONFIGURED:
            break;
        case USBD_EVENT_SET_REMOTE_WAKEUP:
            break;
        case USBD_EVENT_CLR_REMOTE_WAKEUP:
            break;

        default:
            break;
    }
}

void usbd_audio_open(uint8_t intf)
{
    rx_flag = 1;
    /* setup first out ep read transfer */
   int res =  usbd_ep_start_read(AUDIO_OUT_EP, read_buffer, AUDIO_OUT_PACKET);
    bpos = 0;
 //   USB_LOG_RAW("OPEN %d\r\n",res);
    AUDIO_OUT_Start(read_buffer,ABSIZE,SAMPLE_BITS);
}

void usbd_audio_close(uint8_t intf)
{
   // USB_LOG_RAW("CLOSE\r\n");
    rx_flag = 0;
}

void usbd_audio_get_sampling_freq_table(uint8_t ep, uint8_t **sampling_freq_table)
{
    if (ep == AUDIO_OUT_EP)
    {
    	printf("usbd_audio_get_sampling_freq_table\n");
        *sampling_freq_table = (uint8_t *)default_sampling_freq_table;
    }
}

void usbd_audio_iso_out_callback(uint8_t ep, uint32_t nbytes)
{
   // USB_LOG_RAW("actual out len:%d\r\n", nbytes);
    if(ep==AUDIO_OUT_EP)
    {
		int nbpos = (bpos+1)%NP;
		usbd_ep_start_read(AUDIO_OUT_EP, &read_buffer[nbpos*AUDIO_OUT_PACKET], AUDIO_OUT_PACKET);
		AUDIO_OUT_Periodic(&read_buffer[bpos*AUDIO_OUT_PACKET],nbytes);
		bpos = nbpos;
    }
    //usbd_ep_start_read(AUDIO_OUT_EP, read_buffer, AUDIO_OUT_PACKET);
}

#define HID_STATE_IDLE 0
#define HID_STATE_BUSY 1

/*!< hid state ! Data can be sent only when state is idle  */
static volatile uint8_t hid_state = HID_STATE_IDLE;

void usbd_hid_int_callback(uint8_t ep, uint32_t nbytes)
{
    hid_state = HID_STATE_IDLE;
   // printf("callback ep=%d bt=%d\n",ep,nbytes);
    //cr_flush();
}


static struct usbd_endpoint audio_out_ep = {
    .ep_cb = usbd_audio_iso_out_callback,
    .ep_addr = AUDIO_OUT_EP
};

static struct usbd_endpoint hid_in_ep = {
    .ep_cb = usbd_hid_int_callback,
    .ep_addr = HID_INT_EP
};




struct usbd_interface intf0;
struct usbd_interface intf1;
struct usbd_interface intf2;

struct audio_entity_info audio_entity_table[] = {
    { .bEntityId = AUDIO_OUT_CLOCK_ID,
      .bDescriptorSubtype = AUDIO_CONTROL_CLOCK_SOURCE,
      .ep = AUDIO_OUT_EP },
    { .bEntityId = AUDIO_OUT_FU_ID,
      .bDescriptorSubtype = AUDIO_CONTROL_FEATURE_UNIT,
      .ep = AUDIO_OUT_EP },
};



void audio_v2_init(void)
{
	//printf("audio_v2_init start\n");
    usbd_desc_register(audio_v2_descriptor);
    usbd_add_interface(usbd_audio_init_intf(&intf0, 0x0200, audio_entity_table, 2));
    usbd_add_interface(usbd_audio_init_intf(&intf1, 0x0200, audio_entity_table, 2));
    usbd_add_interface(usbd_hid_init_intf(&intf2, hid_keyboard_report_desc, HID_KEYBOARD_REPORT_DESC_SIZE));
   // printf("intf2.intf_num %d %p \n",intf2.intf_num,intf2.hid_report_descriptor);

    usbd_add_endpoint(&hid_in_ep);
    usbd_add_endpoint(&audio_out_ep);

    usbd_initialize();
    printf("AUDIO_OUT_PACKET %d \n",AUDIO_OUT_PACKET);
    //printf("audio_v2_init end\n");
}

void audio_v2_test(void)
{
    if (rx_flag) {
    }
}

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t write_buffer[64];

void hid_keyboard_test(uint8_t busid)
{
    uint8_t sendbuffer[8] = { 0x01, 0x00, 0x00, busid, 0x00, 0x00, 0x00, 0x00 };
	//uint8_t sendbuffer[8] = { 0x01, 0x00, busid, 0x00, 0x00, 0x00, 0x00 ,0x0};
    memcpy(write_buffer, sendbuffer, 8);
    int ret = usbd_ep_start_write(HID_INT_EP, write_buffer, 8);
    if (ret < 0)
    {
    		printf("send err\n");
    		return;
    }
    hid_state = HID_STATE_BUSY;
    int cnt = 20;
    while (hid_state == HID_STATE_BUSY && cnt > 0)
    {
    	HAL_Delay(1);
    	cnt--;
    }
    if(cnt==0)
    {
    	printf("err!\n");
    }
}

void hid_keyboard_test2(uint8_t busid0)
{
	uint8_t sendbuffer[2] = { 0x02, busid0,0x0};
    memcpy(write_buffer, sendbuffer, 3);
    int ret = usbd_ep_start_write(HID_INT_EP, write_buffer, 3);
    if (ret < 0)
    {
    		printf("send err\n");
    		return;
    }
    hid_state = HID_STATE_BUSY;
    int cnt = 20;
    while (hid_state == HID_STATE_BUSY && cnt > 0)
    {
    	HAL_Delay(1);
    	cnt--;
    }

    if(cnt==0)
    {
    	printf("err!\n");
    }
}

