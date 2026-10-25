/*
*********************************************************************************************************
*                                            uC/USB-Device
*
*                                  USB VIDEO CLASS (UVC) - MINIMAL
*
* Filename : usbd_uvc.c
*
* Note(s)  : (1) See 'usbd_uvc.h' for supported subset of the specification.
*
*            (2) Streaming state:
*                (a) SET_CUR(VS_COMMIT_CONTROL) starts streaming.
*                (b) CLEAR_FEATURE(ENDPOINT_HALT) on bulk EP (Linux uvcvideo does it on stream off)
*                    or configuration change stops streaming.
*                Every start/stop aborts the frame in progress, so the next frame starts from its
*                payload header.
*********************************************************************************************************
*/

#define  USBD_UVC_MODULE

#include  "usbd_uvc.h"


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

#define  UVC_SC_VIDEOCONTROL                            0x01u   /* Interface subclass codes.                            */
#define  UVC_SC_VIDEOSTREAMING                          0x02u
#define  UVC_SC_VIDEO_INTERFACE_COLLECTION              0x03u

#define  UVC_CS_INTERFACE                               0x24u   /* Class-specific descriptor type.                      */

#define  UVC_VC_HEADER                                  0x01u   /* VC interface descriptor subtypes.                    */
#define  UVC_VC_INPUT_TERMINAL                          0x02u
#define  UVC_VC_OUTPUT_TERMINAL                         0x03u

#define  UVC_VS_INPUT_HEADER                            0x01u   /* VS interface descriptor subtypes.                    */
#define  UVC_VS_FORMAT_UNCOMPRESSED                     0x04u
#define  UVC_VS_FRAME_UNCOMPRESSED                      0x05u

#define  UVC_ITT_CAMERA                               0x0201u   /* Terminal types.                                      */
#define  UVC_TT_STREAMING                             0x0101u

#define  UVC_IT_ID                                         1u   /* Entity IDs.                                          */
#define  UVC_OT_ID                                         2u

#define  UVC_SET_CUR                                    0x01u   /* Class-specific requests.                             */
#define  UVC_GET_CUR                                    0x81u
#define  UVC_GET_MIN                                    0x82u
#define  UVC_GET_MAX                                    0x83u
#define  UVC_GET_RES                                    0x84u
#define  UVC_GET_LEN                                    0x85u
#define  UVC_GET_INFO                                   0x86u
#define  UVC_GET_DEF                                    0x87u

#define  UVC_VS_PROBE_CONTROL                           0x01u   /* VS interface control selectors.                      */
#define  UVC_VS_COMMIT_CONTROL                          0x02u

#define  UVC_PROBE_LEN                                    26u   /* Probe/Commit ctrl len for UVC 1.0.                   */
#define  UVC_PROBE_BUF_LEN                                48u   /* Room for longer ctrl of newer UVC versions.          */

#define  UVC_HDR_EOH                                    0x80u   /* Payload header bits.                                 */
#define  UVC_HDR_EOF                                    0x02u
#define  UVC_HDR_FID                                    0x01u

#define  UVC_VC_DESC_LEN                       (13u + 18u + 9u) /* Header + IT + OT.                                    */
#define  UVC_VS_DESC_LEN                      (14u + 27u + 30u) /* Input header + format + frame.                       */

#define  UVC_BULK_MAX_PKT_LEN                             64u


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

static  USBD_UVC_FMT          UVC_Fmt;
static  CPU_INT32U            UVC_FrameSize;

static  CPU_INT08U            UVC_DevNbr;
static  CPU_INT08U            UVC_VC_IF_Nbr;
static  CPU_INT08U            UVC_VS_IF_Nbr;
static  CPU_INT08U            UVC_EP_Addr;
static  CPU_BOOLEAN           UVC_Added;

static  volatile  CPU_BOOLEAN  UVC_Conn;
static  volatile  CPU_BOOLEAN  UVC_Streaming;
static  volatile  CPU_INT08U   UVC_StreamSeq;                   /* Changed on every stream start/stop.                  */
static  CPU_INT08U             UVC_FID;

                                                                /* CPU_INT32U arrays: driver accesses FIFO by 16 bits.  */
static  CPU_INT32U            UVC_TxBuf[USBD_UVC_CFG_BUF_LEN / 4u];
static  CPU_INT32U            UVC_CtrlBuf[UVC_PROBE_BUF_LEN / 4u];


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void         USBD_UVC_Conn          (       CPU_INT08U       dev_nbr,
                                                    CPU_INT08U       cfg_nbr,
                                                    void            *p_if_class_arg);

static  void         USBD_UVC_Disconn       (       CPU_INT08U       dev_nbr,
                                                    CPU_INT08U       cfg_nbr,
                                                    void            *p_if_class_arg);

static  void         USBD_UVC_EP_StateUpdate(       CPU_INT08U       dev_nbr,
                                                    CPU_INT08U       cfg_nbr,
                                                    CPU_INT08U       if_nbr,
                                                    CPU_INT08U       if_alt_nbr,
                                                    CPU_INT08U       ep_addr,
                                                    void            *p_if_class_arg,
                                                    void            *p_if_alt_class_arg);

static  void         USBD_UVC_IF_Desc       (       CPU_INT08U       dev_nbr,
                                                    CPU_INT08U       cfg_nbr,
                                                    CPU_INT08U       if_nbr,
                                                    CPU_INT08U       if_alt_nbr,
                                                    void            *p_if_class_arg,
                                                    void            *p_if_alt_class_arg);

static  CPU_INT16U   USBD_UVC_IF_DescSizeGet(       CPU_INT08U       dev_nbr,
                                                    CPU_INT08U       cfg_nbr,
                                                    CPU_INT08U       if_nbr,
                                                    CPU_INT08U       if_alt_nbr,
                                                    void            *p_if_class_arg,
                                                    void            *p_if_alt_class_arg);

static  CPU_BOOLEAN  USBD_UVC_ClassReq      (       CPU_INT08U       dev_nbr,
                                             const  USBD_SETUP_REQ  *p_setup_req,
                                                    void            *p_if_class_arg);

static  void         USBD_UVC_StreamSet     (       CPU_BOOLEAN      on);

static  void         USBD_UVC_ProbeBuild    (       CPU_INT08U      *p_buf);


/*
*********************************************************************************************************
*                                         UVC CLASS DRIVER
*********************************************************************************************************
*/

static  USBD_CLASS_DRV  USBD_UVC_Drv = {
    USBD_UVC_Conn,
    USBD_UVC_Disconn,
    0,                                                          /* No alternate settings.                               */
    USBD_UVC_EP_StateUpdate,
    USBD_UVC_IF_Desc,
    USBD_UVC_IF_DescSizeGet,
    0,                                                          /* No class-specific EP desc.                           */
    0,
    0,                                                          /* No standard IF requests.                             */
    USBD_UVC_ClassReq,
    0,                                                          /* No vendor requests.                                  */
#if (USBD_CFG_MS_OS_DESC_EN == DEF_ENABLED)
    0,
    0,
#endif
};


/*
*********************************************************************************************************
*                                           USBD_UVC_Init()
*
* Description : Initialize UVC class with the single video mode.
*
* Argument(s) : p_fmt       Pointer to video mode description. Copied.
*
*               p_err       Pointer to variable that will receive the return error code.
*********************************************************************************************************
*/

void  USBD_UVC_Init (const  USBD_UVC_FMT  *p_fmt,
                            USBD_ERR      *p_err)
{
    if (p_fmt == (USBD_UVC_FMT *)0) {
       *p_err = USBD_ERR_NULL_PTR;
        return;
    }

    UVC_Fmt       = *p_fmt;
    UVC_FrameSize = (CPU_INT32U)p_fmt->Width * p_fmt->Height * p_fmt->BitsPerPixel / 8u;
    UVC_Added     =  DEF_NO;
    UVC_Conn      =  DEF_NO;
    UVC_Streaming =  DEF_NO;
    UVC_StreamSeq =  0u;
    UVC_FID       =  0u;

   *p_err = USBD_ERR_NONE;
}


/*
*********************************************************************************************************
*                                          USBD_UVC_CfgAdd()
*
* Description : Add VC and VS interfaces (grouped by IAD) and bulk IN endpoint to configuration.
*
* Argument(s) : dev_nbr     Device number.
*
*               cfg_nbr     Configuration number.
*
*               p_err       Pointer to variable that will receive the return error code.
*********************************************************************************************************
*/

void  USBD_UVC_CfgAdd (CPU_INT08U   dev_nbr,
                       CPU_INT08U   cfg_nbr,
                       USBD_ERR    *p_err)
{
    if (UVC_Added == DEF_YES) {                                 /* See 'usbd_uvc.h' Note #2.                            */
       *p_err = USBD_ERR_ALLOC;
        return;
    }

    UVC_VC_IF_Nbr = USBD_IF_Add(dev_nbr, cfg_nbr, &USBD_UVC_Drv, (void *)0, (void *)0,
                                USBD_CLASS_CODE_VIDEO, UVC_SC_VIDEOCONTROL, 0u,
                                "UVC Control", p_err);
    if (*p_err != USBD_ERR_NONE) {
        return;
    }

    UVC_VS_IF_Nbr = USBD_IF_Add(dev_nbr, cfg_nbr, &USBD_UVC_Drv, (void *)0, (void *)0,
                                USBD_CLASS_CODE_VIDEO, UVC_SC_VIDEOSTREAMING, 0u,
                                "UVC Streaming", p_err);
    if (*p_err != USBD_ERR_NONE) {
        return;
    }

    (void)USBD_IF_Grp(dev_nbr, cfg_nbr,
                      USBD_CLASS_CODE_VIDEO, UVC_SC_VIDEO_INTERFACE_COLLECTION, 0u,
                      UVC_VC_IF_Nbr, 2u,
                      "UVC Camera", p_err);
    if (*p_err != USBD_ERR_NONE) {
        return;
    }

    UVC_EP_Addr = USBD_BulkAdd(dev_nbr, cfg_nbr, UVC_VS_IF_Nbr, 0u,
                               DEF_YES, UVC_BULK_MAX_PKT_LEN, p_err);
    if (*p_err != USBD_ERR_NONE) {
        return;
    }

    UVC_DevNbr = dev_nbr;
    UVC_Added  = DEF_YES;
}


/*
*********************************************************************************************************
*                                          USBD_UVC_IsConn()
*
* Description : Return DEF_YES if device is in configured state.
*********************************************************************************************************
*/

CPU_BOOLEAN  USBD_UVC_IsConn (void)
{
    return (UVC_Conn);
}


/*
*********************************************************************************************************
*                                       USBD_UVC_IsStreaming()
*
* Description : Return DEF_YES if host has committed streaming parameters and has not stopped stream.
*********************************************************************************************************
*/

CPU_BOOLEAN  USBD_UVC_IsStreaming (void)
{
    return ((UVC_Conn == DEF_YES) && (UVC_Streaming == DEF_YES));
}


/*
*********************************************************************************************************
*                                          USBD_UVC_FrameTx()
*
* Description : Send one video frame as one payload. Blocks until the whole frame is sent.
*
* Argument(s) : fill_fnct   Function that generates frame data, see USBD_UVC_FILL_FNCT.
*
*               p_arg       Argument passed to 'fill_fnct'.
*
*               p_err       Pointer to variable that will receive the return error code:
*
*                               USBD_ERR_NONE                   Frame sent.
*                               USBD_ERR_INVALID_CLASS_STATE    Not streaming, or stream was restarted.
*
*                               - RETURNED BY USBD_BulkTx() -
*
* Note(s)     : (1) Frame is generated by small chunks, so no frame buffer is needed. All chunks
*                   except the last one are multiples of max packet size, so the host sees one
*                   continuous bulk transfer.
*********************************************************************************************************
*/

void  USBD_UVC_FrameTx (USBD_UVC_FILL_FNCT   fill_fnct,
                        void                *p_arg,
                        USBD_ERR            *p_err)
{
    CPU_INT08U   *p_buf;
    CPU_INT08U    seq;
    CPU_INT32U    total;
    CPU_INT32U    pos;
    CPU_INT32U    len;
    CPU_INT32U    hdr;
    CPU_BOOLEAN   end;


    p_buf = (CPU_INT08U *)&UVC_TxBuf[0];
    seq   =  UVC_StreamSeq;
    total =  USBD_UVC_PAYLOAD_HDR_LEN + UVC_FrameSize;
    pos   =  0u;

    UVC_FID ^= UVC_HDR_FID;                                     /* FID toggles on every new frame.                      */

    while (pos < total) {
        if ((USBD_UVC_IsStreaming() != DEF_YES) ||
            (seq                    != UVC_StreamSeq)) {
           *p_err = USBD_ERR_INVALID_CLASS_STATE;
            return;
        }

        len = DEF_MIN(USBD_UVC_CFG_BUF_LEN, total - pos);
        hdr = 0u;
        if (pos == 0u) {
            p_buf[0] = USBD_UVC_PAYLOAD_HDR_LEN;
            p_buf[1] = UVC_HDR_EOH | UVC_HDR_EOF | UVC_FID;     /* Whole frame in one payload: EOF in first header.     */
            hdr      = USBD_UVC_PAYLOAD_HDR_LEN;
        }

        fill_fnct(p_arg,
                  pos + hdr - USBD_UVC_PAYLOAD_HDR_LEN,         /* Offset in frame data.                                */
                 &p_buf[hdr],
                  len - hdr);

        end = (pos + len == total) ? DEF_YES : DEF_NO;
        (void)USBD_BulkTx(UVC_DevNbr, UVC_EP_Addr, p_buf, len,
                          USBD_UVC_CFG_TX_TIMEOUT_mS, end, p_err);
        if (*p_err != USBD_ERR_NONE) {
            if ((USBD_UVC_IsStreaming() != DEF_YES) ||          /* Tx aborted by stream stop/restart: not an error.     */
                (seq                    != UVC_StreamSeq)) {
               *p_err = USBD_ERR_INVALID_CLASS_STATE;
            }
            return;
        }

        pos += len;
    }

   *p_err = USBD_ERR_NONE;
}


/*
*********************************************************************************************************
*                                          USBD_UVC_Conn()
*                                         USBD_UVC_Disconn()
*
* Description : Configuration activated / deactivated. Called for each interface (VC and VS).
*********************************************************************************************************
*/

static  void  USBD_UVC_Conn (CPU_INT08U   dev_nbr,
                             CPU_INT08U   cfg_nbr,
                             void        *p_if_class_arg)
{
    (void)dev_nbr;
    (void)cfg_nbr;
    (void)p_if_class_arg;

    UVC_Streaming = DEF_NO;
    UVC_StreamSeq++;
    UVC_Conn      = DEF_YES;
}


static  void  USBD_UVC_Disconn (CPU_INT08U   dev_nbr,
                                CPU_INT08U   cfg_nbr,
                                void        *p_if_class_arg)
{
    (void)dev_nbr;
    (void)cfg_nbr;
    (void)p_if_class_arg;

    UVC_Conn      = DEF_NO;
    UVC_Streaming = DEF_NO;
    UVC_StreamSeq++;
}


/*
*********************************************************************************************************
*                                      USBD_UVC_EP_StateUpdate()
*
* Description : Called on SET/CLEAR_FEATURE(ENDPOINT_HALT). For bulk streaming the host stops
*               the stream with CLEAR_FEATURE(ENDPOINT_HALT).
*********************************************************************************************************
*/

static  void  USBD_UVC_EP_StateUpdate (CPU_INT08U   dev_nbr,
                                       CPU_INT08U   cfg_nbr,
                                       CPU_INT08U   if_nbr,
                                       CPU_INT08U   if_alt_nbr,
                                       CPU_INT08U   ep_addr,
                                       void        *p_if_class_arg,
                                       void        *p_if_alt_class_arg)
{
    (void)dev_nbr;
    (void)cfg_nbr;
    (void)if_nbr;
    (void)if_alt_nbr;
    (void)p_if_class_arg;
    (void)p_if_alt_class_arg;

    if (ep_addr == UVC_EP_Addr) {
        USBD_UVC_StreamSet(DEF_OFF);
    }
}


/*
*********************************************************************************************************
*                                         USBD_UVC_StreamSet()
*
* Description : Start/stop stream. Aborts transfer in progress, so streaming task is released
*               immediately and FIFO does not keep data of the interrupted frame.
*********************************************************************************************************
*/

static  void  USBD_UVC_StreamSet (CPU_BOOLEAN  on)
{
    USBD_ERR  err;


    UVC_Streaming = on;
    UVC_StreamSeq++;
    USBD_EP_Abort(UVC_DevNbr, UVC_EP_Addr, &err);
    (void)err;
}


/*
*********************************************************************************************************
*                                         USBD_UVC_IF_Desc()
*
* Description : Write class-specific interface descriptors (placed after standard IF desc).
*********************************************************************************************************
*/

static  void  USBD_UVC_IF_Desc (CPU_INT08U   dev_nbr,
                                CPU_INT08U   cfg_nbr,
                                CPU_INT08U   if_nbr,
                                CPU_INT08U   if_alt_nbr,
                                void        *p_if_class_arg,
                                void        *p_if_alt_class_arg)
{
    CPU_INT32U  fps;
    CPU_INT32U  bit_rate;


    (void)cfg_nbr;
    (void)if_alt_nbr;
    (void)p_if_class_arg;
    (void)p_if_alt_class_arg;

    if (if_nbr == UVC_VC_IF_Nbr) {
                                                                /* ---------------- VC HEADER (13 bytes) -------------- */
        USBD_DescWr08(dev_nbr, 13u);
        USBD_DescWr08(dev_nbr, UVC_CS_INTERFACE);
        USBD_DescWr08(dev_nbr, UVC_VC_HEADER);
        USBD_DescWr16(dev_nbr, 0x0100u);                        /* bcdUVC 1.00.                                         */
        USBD_DescWr16(dev_nbr, UVC_VC_DESC_LEN);                /* wTotalLength.                                        */
        USBD_DescWr32(dev_nbr, 48000000u);                      /* dwClockFrequency (deprecated).                       */
        USBD_DescWr08(dev_nbr, 1u);                             /* bInCollection.                                       */
        USBD_DescWr08(dev_nbr, UVC_VS_IF_Nbr);                  /* baInterfaceNr(1).                                    */
                                                                /* ------------ CAMERA INPUT TERMINAL (18) ------------ */
        USBD_DescWr08(dev_nbr, 18u);
        USBD_DescWr08(dev_nbr, UVC_CS_INTERFACE);
        USBD_DescWr08(dev_nbr, UVC_VC_INPUT_TERMINAL);
        USBD_DescWr08(dev_nbr, UVC_IT_ID);
        USBD_DescWr16(dev_nbr, UVC_ITT_CAMERA);
        USBD_DescWr08(dev_nbr, 0u);                             /* bAssocTerminal.                                      */
        USBD_DescWr08(dev_nbr, 0u);                             /* iTerminal.                                           */
        USBD_DescWr16(dev_nbr, 0u);                             /* wObjectiveFocalLengthMin.                            */
        USBD_DescWr16(dev_nbr, 0u);                             /* wObjectiveFocalLengthMax.                            */
        USBD_DescWr16(dev_nbr, 0u);                             /* wOcularFocalLength.                                  */
        USBD_DescWr08(dev_nbr, 3u);                             /* bControlSize.                                        */
        USBD_DescWr24(dev_nbr, 0u);                             /* bmControls: none.                                    */
                                                                /* --------------- OUTPUT TERMINAL (9) ---------------- */
        USBD_DescWr08(dev_nbr, 9u);
        USBD_DescWr08(dev_nbr, UVC_CS_INTERFACE);
        USBD_DescWr08(dev_nbr, UVC_VC_OUTPUT_TERMINAL);
        USBD_DescWr08(dev_nbr, UVC_OT_ID);
        USBD_DescWr16(dev_nbr, UVC_TT_STREAMING);
        USBD_DescWr08(dev_nbr, 0u);                             /* bAssocTerminal.                                      */
        USBD_DescWr08(dev_nbr, UVC_IT_ID);                      /* bSourceID.                                           */
        USBD_DescWr08(dev_nbr, 0u);                             /* iTerminal.                                           */

    } else if (if_nbr == UVC_VS_IF_Nbr) {
        fps      = 10000000u / UVC_Fmt.FrameInterval;
        bit_rate = UVC_FrameSize * 8u * fps;
                                                                /* ------------- VS INPUT HEADER (14 bytes) ----------- */
        USBD_DescWr08(dev_nbr, 14u);
        USBD_DescWr08(dev_nbr, UVC_CS_INTERFACE);
        USBD_DescWr08(dev_nbr, UVC_VS_INPUT_HEADER);
        USBD_DescWr08(dev_nbr, 1u);                             /* bNumFormats.                                         */
        USBD_DescWr16(dev_nbr, UVC_VS_DESC_LEN);                /* wTotalLength.                                        */
        USBD_DescWr08(dev_nbr, UVC_EP_Addr);                    /* bEndpointAddress.                                    */
        USBD_DescWr08(dev_nbr, 0u);                             /* bmInfo.                                              */
        USBD_DescWr08(dev_nbr, UVC_OT_ID);                      /* bTerminalLink.                                       */
        USBD_DescWr08(dev_nbr, 0u);                             /* bStillCaptureMethod.                                 */
        USBD_DescWr08(dev_nbr, 0u);                             /* bTriggerSupport.                                     */
        USBD_DescWr08(dev_nbr, 0u);                             /* bTriggerUsage.                                       */
        USBD_DescWr08(dev_nbr, 1u);                             /* bControlSize.                                        */
        USBD_DescWr08(dev_nbr, 0u);                             /* bmaControls(1).                                      */
                                                                /* -------- UNCOMPRESSED FORMAT (27 bytes) ------------ */
        USBD_DescWr08(dev_nbr, 27u);
        USBD_DescWr08(dev_nbr, UVC_CS_INTERFACE);
        USBD_DescWr08(dev_nbr, UVC_VS_FORMAT_UNCOMPRESSED);
        USBD_DescWr08(dev_nbr, 1u);                             /* bFormatIndex.                                        */
        USBD_DescWr08(dev_nbr, 1u);                             /* bNumFrameDescriptors.                                */
        USBD_DescWr  (dev_nbr, UVC_Fmt.Guid, 16u);              /* guidFormat.                                          */
        USBD_DescWr08(dev_nbr, UVC_Fmt.BitsPerPixel);
        USBD_DescWr08(dev_nbr, 1u);                             /* bDefaultFrameIndex.                                  */
        USBD_DescWr08(dev_nbr, 0u);                             /* bAspectRatioX.                                       */
        USBD_DescWr08(dev_nbr, 0u);                             /* bAspectRatioY.                                       */
        USBD_DescWr08(dev_nbr, 0u);                             /* bmInterlaceFlags.                                    */
        USBD_DescWr08(dev_nbr, 0u);                             /* bCopyProtect.                                        */
                                                                /* --------- UNCOMPRESSED FRAME (30 bytes) ------------ */
        USBD_DescWr08(dev_nbr, 30u);
        USBD_DescWr08(dev_nbr, UVC_CS_INTERFACE);
        USBD_DescWr08(dev_nbr, UVC_VS_FRAME_UNCOMPRESSED);
        USBD_DescWr08(dev_nbr, 1u);                             /* bFrameIndex.                                         */
        USBD_DescWr08(dev_nbr, 0u);                             /* bmCapabilities.                                      */
        USBD_DescWr16(dev_nbr, UVC_Fmt.Width);
        USBD_DescWr16(dev_nbr, UVC_Fmt.Height);
        USBD_DescWr32(dev_nbr, bit_rate);                       /* dwMinBitRate.                                        */
        USBD_DescWr32(dev_nbr, bit_rate);                       /* dwMaxBitRate.                                        */
        USBD_DescWr32(dev_nbr, UVC_FrameSize);                  /* dwMaxVideoFrameBufferSize.                           */
        USBD_DescWr32(dev_nbr, UVC_Fmt.FrameInterval);          /* dwDefaultFrameInterval.                              */
        USBD_DescWr08(dev_nbr, 1u);                             /* bFrameIntervalType: 1 discrete interval.             */
        USBD_DescWr32(dev_nbr, UVC_Fmt.FrameInterval);          /* dwFrameInterval(1).                                  */
    }
}


static  CPU_INT16U  USBD_UVC_IF_DescSizeGet (CPU_INT08U   dev_nbr,
                                             CPU_INT08U   cfg_nbr,
                                             CPU_INT08U   if_nbr,
                                             CPU_INT08U   if_alt_nbr,
                                             void        *p_if_class_arg,
                                             void        *p_if_alt_class_arg)
{
    (void)dev_nbr;
    (void)cfg_nbr;
    (void)if_alt_nbr;
    (void)p_if_class_arg;
    (void)p_if_alt_class_arg;

    if (if_nbr == UVC_VC_IF_Nbr) {
        return (UVC_VC_DESC_LEN);
    }
    if (if_nbr == UVC_VS_IF_Nbr) {
        return (UVC_VS_DESC_LEN);
    }
    return (0u);
}


/*
*********************************************************************************************************
*                                        USBD_UVC_ProbeBuild()
*
* Description : Build Probe/Commit control (UVC 1.0, 26 bytes) describing the single video mode.
*********************************************************************************************************
*/

static  void  USBD_UVC_ProbeBuild (CPU_INT08U  *p_buf)
{
    CPU_INT32U  payload;


    payload = USBD_UVC_PAYLOAD_HDR_LEN + UVC_FrameSize;

    Mem_Clr(p_buf, UVC_PROBE_LEN);
    p_buf[2]  = 1u;                                             /* bFormatIndex.                                        */
    p_buf[3]  = 1u;                                             /* bFrameIndex.                                         */
    MEM_VAL_SET_INT32U_LITTLE(&p_buf[4],  UVC_Fmt.FrameInterval);
    MEM_VAL_SET_INT32U_LITTLE(&p_buf[18], UVC_FrameSize);       /* dwMaxVideoFrameSize.                                 */
    MEM_VAL_SET_INT32U_LITTLE(&p_buf[22], payload);             /* dwMaxPayloadTransferSize.                            */
}


/*
*********************************************************************************************************
*                                         USBD_UVC_ClassReq()
*
* Description : Class-specific requests. Only VS Probe and Commit controls are supported, everything
*               else is stalled.
*********************************************************************************************************
*/

static  CPU_BOOLEAN  USBD_UVC_ClassReq (       CPU_INT08U       dev_nbr,
                                        const  USBD_SETUP_REQ  *p_setup_req,
                                               void            *p_if_class_arg)
{
    CPU_INT08U  *p_buf;
    CPU_INT08U   cs;
    CPU_INT16U   len;
    USBD_ERR     err;


    (void)p_if_class_arg;

    p_buf = (CPU_INT08U *)&UVC_CtrlBuf[0];
    cs    = (CPU_INT08U)(p_setup_req->wValue >> 8u);

    if (((p_setup_req->bmRequestType & USBD_REQ_RECIPIENT_MASK) != USBD_REQ_RECIPIENT_INTERFACE) ||
        ((p_setup_req->wIndex & 0xFFu)                         != UVC_VS_IF_Nbr)                 ||
        ((cs != UVC_VS_PROBE_CONTROL) && (cs != UVC_VS_COMMIT_CONTROL))) {
        return (DEF_FAIL);
    }

    switch (p_setup_req->bRequest) {
        case UVC_SET_CUR:                                       /* Host values are ignored: only one mode exists.       */
             if (p_setup_req->wLength > UVC_PROBE_BUF_LEN) {
                 return (DEF_FAIL);
             }
             (void)USBD_CtrlRx(dev_nbr, p_buf, p_setup_req->wLength,
                               USBD_UVC_CFG_CTRL_REQ_TIMEOUT_mS, &err);
             if (err != USBD_ERR_NONE) {
                 return (DEF_FAIL);
             }
             if (cs == UVC_VS_COMMIT_CONTROL) {
                 USBD_UVC_StreamSet(DEF_ON);
             }
             return (DEF_OK);


        case UVC_GET_CUR:
        case UVC_GET_MIN:
        case UVC_GET_MAX:
        case UVC_GET_DEF:
             USBD_UVC_ProbeBuild(p_buf);
             len = DEF_MIN(p_setup_req->wLength, UVC_PROBE_LEN);
             break;


        case UVC_GET_RES:
             Mem_Clr(p_buf, UVC_PROBE_LEN);
             len = DEF_MIN(p_setup_req->wLength, UVC_PROBE_LEN);
             break;


        case UVC_GET_LEN:
             p_buf[0] = UVC_PROBE_LEN;
             p_buf[1] = 0u;
             len = DEF_MIN(p_setup_req->wLength, 2u);
             break;


        case UVC_GET_INFO:
             p_buf[0] = 0x03u;                                  /* Supports GET and SET.                                */
             len = DEF_MIN(p_setup_req->wLength, 1u);
             break;


        default:
             return (DEF_FAIL);
    }

    (void)USBD_CtrlTx(dev_nbr, p_buf, len,
                      USBD_UVC_CFG_CTRL_REQ_TIMEOUT_mS, DEF_NO, &err);

    return ((err == USBD_ERR_NONE) ? DEF_OK : DEF_FAIL);
}
