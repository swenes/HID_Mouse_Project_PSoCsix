
#include "project.h"
#include "cyapicallbacks.h"

#define MOUSE_ENDPOINT (1u)
#define MOUSE_DATA_LEN (3U)
#define CURSOR_STEP_PLUS ((uint8_t)(10))
#define CURSOR_STEP_MINUS ((uint8_t)(-10))
#define CURSOR_DELAY (128U)


uint32_t CURSOR_STEP_POS = (1U << 6); // sensörlerin orta noktasını bulduk. byte shift yöntemi ile.
bool leftButtonClickPending = false;
bool rightButtonClickPending = false;
bool horizontalModeEnabled = false;
bool previousButtonState = false;
bool currentButtonState = 0;


CY_USB_DEV_ALLOC_ENDPOINT_BUFFER(mouseData, MOUSE_DATA_LEN);

static void USBFS_IsrHigh(void);
static void USBFS_IsrMedium(void);
static void USBFS_IsrLow(void);
bool isEnabled(bool buttonNameMode,GPIO_PRT_Type* port, uint8_t pin);


const cy_stc_sysint_t UsbDevIntrHigh =
    {
        .intrSrc = (IRQn_Type)usb_interrupt_hi_IRQn,
        .intrPriority = 5U,
};
const cy_stc_sysint_t UsbDevIntrMedium =
    {
        .intrSrc = (IRQn_Type)usb_interrupt_med_IRQn,
        .intrPriority = 6U,
};
const cy_stc_sysint_t UsbDevIntrLow =
    {
        .intrSrc = (IRQn_Type)usb_interrupt_lo_IRQn,
        .intrPriority = 7U,
};

int main(void)
{
    cy_en_usb_dev_status_t status;

    status = Cy_USB_Dev_Init(USBFS_HW, &USBFS_drvConfig, &USBFS_drvContext, &USBFS_devices[0], &USBFS_devConfig, &USBFS_devContext);
    if (CY_USB_DEV_SUCCESS != status)
    {
        while (1);
    }

    status = Cy_USB_Dev_HID_Init(&USBFS_hidConfig, &USBFS_hidContext, &USBFS_devContext);
    if (CY_USB_DEV_SUCCESS != status)
    {
        while (1);
    }

    (void)Cy_SysInt_Init(&UsbDevIntrHigh, &USBFS_IsrHigh);
    (void)Cy_SysInt_Init(&UsbDevIntrMedium, &USBFS_IsrMedium);
    (void)Cy_SysInt_Init(&UsbDevIntrLow, &USBFS_IsrLow);

    NVIC_EnableIRQ(UsbDevIntrHigh.intrSrc);
    NVIC_EnableIRQ(UsbDevIntrMedium.intrSrc);
    NVIC_EnableIRQ(UsbDevIntrLow.intrSrc);

    __enable_irq();
    CapSense_Start();
    CapSense_ScanAllWidgets();
    PWM_1_Start();

    Cy_USB_Dev_Connect(true, CY_USB_DEV_WAIT_FOREVER, &USBFS_devContext);
    
    
    for (;;)
    {
        horizontalModeEnabled = isEnabled(horizontalModeEnabled,P0_4_PORT,P0_4_NUM); //kill switch enabled ya da disabled dönüyorum.
        
        if (!CapSense_IsBusy()) // CapSense sensörünün meşgul olup olmadığını ve killSwitchi kontrol ediyoruz.
        {
            //horizontal true durumuna göre ledleri ayarladık.
            Cy_GPIO_Write(P11_1_PORT, P11_1_NUM, horizontalModeEnabled ? 0 : 1); 
            Cy_GPIO_Write(P0_3_PORT, P0_3_NUM, horizontalModeEnabled ? 1 : 0);

            // Tüm widgetları işledik.
            CapSense_ProcessAllWidgets();
            
            bool sliderTouched = CapSense_IsWidgetActive(CapSense_LINEARSLIDER0_WDGT_ID); // slidera dokunuluyor mu?
            
            if (sliderTouched)
            {
                // önceki pozisyon ve şimdiki pozisyona göre imlece gerekli değerleri atayacağız.
                uint32 prevPos;
                uint32 pos = CapSense_GetCentroidPos(CapSense_LINEARSLIDER0_WDGT_ID); // pos. al
                if (pos < prevPos)
                {
                   mouseData[horizontalModeEnabled == 0 ? 1 : 2] = CURSOR_STEP_MINUS; // sola ya da aşağı kaydırma.
                }
                else if (pos > prevPos)
                {
                    mouseData[horizontalModeEnabled == 0 ? 1 : 2] = CURSOR_STEP_PLUS; // sağa ya da yukarı kaydırma
                }
                else
                {
                    mouseData[horizontalModeEnabled == 0 ? 1 : 2] = 0; // iki pozisyon da eşit ise hareket etme
                }

                leftButtonClickPending = CapSense_IsWidgetActive(CapSense_BUTTON0_WDGT_ID);  // sol click aktif mi kontrolü
                rightButtonClickPending = CapSense_IsWidgetActive(CapSense_BUTTON1_WDGT_ID); // sağ click aktif mi kontrolü
                
                // usb data transfer işlemi
                Cy_USB_Dev_WriteEpBlocking(MOUSE_ENDPOINT, mouseData, MOUSE_DATA_LEN, CY_USB_DEV_WAIT_FOREVER, &USBFS_devContext);

                prevPos = pos; // şuanki posu önceki posa eşitliyoruz
            }

            else
            {
                leftButtonClickPending = CapSense_IsWidgetActive(CapSense_BUTTON0_WDGT_ID);
                rightButtonClickPending = CapSense_IsWidgetActive(CapSense_BUTTON1_WDGT_ID);

                mouseData[0] = leftButtonClickPending ? 1 : (rightButtonClickPending ? 2 : 0); // sol clik ise 1, sağ ise 2, ne sol ne sağ ise 0 atıyorum.
                mouseData[horizontalModeEnabled == 0 ?  1 : 2] = 0;  // imleç pozisyonu sabit

                Cy_USB_Dev_WriteEpBlocking(MOUSE_ENDPOINT, mouseData, MOUSE_DATA_LEN, CY_USB_DEV_WAIT_FOREVER, &USBFS_devContext); // usb data transferi
            }

            CapSense_UpdateAllBaselines(); // alt hücreleri güncelledik
            CapSense_ScanAllWidgets();    // bütün widgetler tekrar taranıyor ve döngü başa dönüyor.
        }
    }
}
        
bool isEnabled(bool buttonNameMode,GPIO_PRT_Type* port, uint8_t pin)
{
    bool currentButtonState = Cy_GPIO_Read(port, pin);
    
     if (currentButtonState != previousButtonState)
        {
            if (currentButtonState == 1) // Düğme basıldıysa
            {
                buttonNameMode = !buttonNameMode; // Durumu tersine çevir
            }

            previousButtonState = currentButtonState;
        }
    
    return buttonNameMode;
}

static void USBFS_IsrHigh(void)
{
    Cy_USBFS_Dev_Drv_Interrupt(USBFS_HW,
                               Cy_USBFS_Dev_Drv_GetInterruptCauseHi(USBFS_HW),
                               &USBFS_drvContext);
}

static void USBFS_IsrMedium(void)
{
    Cy_USBFS_Dev_Drv_Interrupt(USBFS_HW,
                               Cy_USBFS_Dev_Drv_GetInterruptCauseMed(USBFS_HW),
                               &USBFS_drvContext);
}

static void USBFS_IsrLow(void)
{
    /* Call interrupt processing */
    Cy_USBFS_Dev_Drv_Interrupt(USBFS_HW,
                               Cy_USBFS_Dev_Drv_GetInterruptCauseLo(USBFS_HW),
                               &USBFS_drvContext);
}
