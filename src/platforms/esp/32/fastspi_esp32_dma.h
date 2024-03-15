#pragma once



#include "driver/spi_master.h"
#include "esp_log.h"

class ESP32DMAController
{
public:
  ESP32DMAController(uint8_t DATA_PIN, uint8_t CLOCK_PIN, uint32_t SPI_SPEED, uint32_t numLeds);
    // -- Get or create the pixel data buffer for DMA
    uint8_t *getPixelBuffer(int size_in_bytes);

    // the good stuff... we actually send stuff out
		void IRAM_ATTR showPixels();

  private:
};

template <int DATA_PIN, int CLOCK_PIN, EOrder RGB_ORDER = RGB, uint32_t SPI_SPEED>
class DMAController : public CPixelLEDController<RGB_ORDER>
{
  private:
    // -- The actual controller object for ESP32
    ESP32DMAController mDMAController;

    // -- Verify that the pin is valid
    static_assert(FastPin<DATA_PIN>::validpin(), "Invalid pin specified");
    static_assert(FastPin<CLOCK_PIN>::validpin(), "Invalid pin specified");

  public:
    DMAController() : ESP32DMAController(DATA_PIN, CLOCK_PIN, SPI_SPEED)
    {
    }

    void init()
    {
    }

    virtual uint16_t getMaxRefreshRate() const
    {
        return 400;
    }

  protected:
    // -- Load pixel data
    //    This method loads all of the pixel data into a separate buffer for use by
    //    by the RMT driver. Copying does two important jobs: it fixes the color
    //    order for the pixels, and it performs the scaling/adjusting ahead of time.
    //    It also packs the bytes into 32 bit chunks with the right bit order.
    void loadPixelData(PixelController<RGB_ORDER> &pixels)
    {
        // -- Make sure the buffer is allocated
        int size_in_bytes = pixels.size() * 3;
        uint8_t *pData = mRMTController.getPixelBuffer(size_in_bytes);

        // -- This might be faster
        while (pixels.has(1))
        {
            *pData++ = pixels.loadAndScale0();
            *pData++ = pixels.loadAndScale1();
            *pData++ = pixels.loadAndScale2();
            pixels.advanceData();
            pixels.stepDithering();
        }
    }

    // -- Show pixels
    //    This is the main entry point for the controller.
    virtual void showPixels(PixelController<RGB_ORDER> &pixels)
    {
        if (FASTLED_RMT_BUILTIN_DRIVER)
        {
            convertAllPixelData(pixels);
        }
        else
        {
            loadPixelData(pixels);
        }

        mRMTController.showPixels();
    }

    // -- Convert all pixels to RMT pulses
    //    This function is only used when the user chooses to use the
    //    built-in RMT driver, which needs all of the RMT pulses
    //    up-front.
    void convertAllPixelData(PixelController<RGB_ORDER> &pixels)
    {
        // -- Make sure the data buffer is allocated
        mRMTController.initPulseBuffer(pixels.size() * 3);

        // -- Cycle through the R,G, and B values in the right order,
        //    storing the pulses in the big buffer

        uint32_t byteval;
        while (pixels.has(1))
        {
            byteval = pixels.loadAndScale0();
            mRMTController.convertByte(byteval);
            byteval = pixels.loadAndScale1();
            mRMTController.convertByte(byteval);
            byteval = pixels.loadAndScale2();
            mRMTController.convertByte(byteval);
            pixels.advanceData();
            pixels.stepDithering();
        }
    }
};