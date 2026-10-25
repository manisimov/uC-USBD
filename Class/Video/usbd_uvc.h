/*
*********************************************************************************************************
*                                            uC/USB-Device
*
*                                  USB VIDEO CLASS (UVC) - MINIMAL
*
* Filename : usbd_uvc.h
*
* Note(s)  : (1) Minimal subset of USB Video Class 1.0:
*                (a) One Video Control (VC) interface without interrupt endpoint. Topology is
*                    Camera Input Terminal -> Output Terminal (USB streaming), no units, no controls.
*                (b) One Video Streaming (VS) interface with single bulk IN endpoint (alternate
*                    setting 0 only, so no isochronous support is needed).
*                (c) One uncompressed format with one frame size and one frame interval.
*                (d) Probe / Commit controls always return the single supported mode.
*                (e) Every video frame is sent as one payload: 2 bytes header + frame data.
*                    Payload ends with short packet, so frame size + 2 must not be a multiple of
*                    max packet size (64).
*
*            (2) Only one class instance in one configuration is supported.
*********************************************************************************************************
*/

#ifndef  USBD_UVC_MODULE_PRESENT
#define  USBD_UVC_MODULE_PRESENT


/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include  "../../Source/usbd_core.h"


/*
*********************************************************************************************************
*                                         DEFAULT CONFIGURATION
*********************************************************************************************************
*/

#ifndef  USBD_UVC_CFG_BUF_LEN                                   /* Tx buf len. Must be a multiple of max pkt size.      */
#define  USBD_UVC_CFG_BUF_LEN                            512u
#endif

#ifndef  USBD_UVC_CFG_TX_TIMEOUT_mS                             /* Timeout of one buf tx on bulk EP.                    */
#define  USBD_UVC_CFG_TX_TIMEOUT_mS                      500u
#endif

#ifndef  USBD_UVC_CFG_CTRL_REQ_TIMEOUT_mS
#define  USBD_UVC_CFG_CTRL_REQ_TIMEOUT_mS                500u
#endif


/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/

#define  USBD_UVC_PAYLOAD_HDR_LEN                          2u

                                                                /* Uncompressed format GUIDs ('YUY2', 'Y800').          */
#define  USBD_UVC_GUID_YUY2     0x59u, 0x55u, 0x59u, 0x32u, 0x00u, 0x00u, 0x10u, 0x00u, \
                                0x80u, 0x00u, 0x00u, 0xAAu, 0x00u, 0x38u, 0x9Bu, 0x71u
#define  USBD_UVC_GUID_Y800     0x59u, 0x38u, 0x30u, 0x30u, 0x00u, 0x00u, 0x10u, 0x00u, \
                                0x80u, 0x00u, 0x00u, 0xAAu, 0x00u, 0x38u, 0x9Bu, 0x71u


/*
*********************************************************************************************************
*                                             DATA TYPES
*********************************************************************************************************
*/

typedef  struct  usbd_uvc_fmt {
    CPU_INT08U  Guid[16];                                       /* Uncompressed format GUID.                            */
    CPU_INT08U  BitsPerPixel;
    CPU_INT16U  Width;
    CPU_INT16U  Height;
    CPU_INT32U  FrameInterval;                                  /* Frame interval in 100 ns units.                      */
} USBD_UVC_FMT;

                                                                /* Fill 'len' bytes of frame starting at 'offset'.      */
typedef  void  (*USBD_UVC_FILL_FNCT)(void        *p_arg,
                                     CPU_INT32U   offset,
                                     CPU_INT08U  *p_buf,
                                     CPU_INT32U   len);


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void         USBD_UVC_Init       (const  USBD_UVC_FMT        *p_fmt,
                                         USBD_ERR            *p_err);

void         USBD_UVC_CfgAdd     (       CPU_INT08U           dev_nbr,
                                         CPU_INT08U           cfg_nbr,
                                         USBD_ERR            *p_err);

CPU_BOOLEAN  USBD_UVC_IsConn     (       void);

CPU_BOOLEAN  USBD_UVC_IsStreaming(       void);

void         USBD_UVC_FrameTx    (       USBD_UVC_FILL_FNCT   fill_fnct,
                                         void                *p_arg,
                                         USBD_ERR            *p_err);

#endif
