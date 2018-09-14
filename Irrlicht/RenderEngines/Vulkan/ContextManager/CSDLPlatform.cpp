#include "CSDLPlatform.h"
#include "CVulkanDriver.h"
#include "CVulkanDevice.h"
#include "CVulkanSwapChain.h"
#include "CIrrDeviceSDL.h"

#ifdef _IRR_COMPILE_WITH_SDL_DEVICE_

#ifdef _WIN32
#pragma comment(linker, "/subsystem:windows")
#define VK_USE_PLATFORM_WIN32_KHR
#define PLATFORM_SURFACE_EXTENSION_NAME VK_KHR_WIN32_SURFACE_EXTENSION_NAME
#define PlatformSurfaceCreateInfo VkWin32SurfaceCreateInfoKHR
#define PLATFORM_SURFACE_CREATE_INFO VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR;
#define PLATFORM_ASSIGN_SURFACEINFO \
    createInfo.hinstance = GetModuleHandle(NULL); \
    createInfo.hwnd = info.info.win.window
#define PlatformCreateSurface vkCreateWin32SurfaceKHR
#else /* _WIN32 */
#define VK_USE_PLATFORM_XLIB_KHR
#define PLATFORM_SURFACE_EXTENSION_NAME VK_KHR_XLIB_SURFACE_EXTENSION_NAME
#define PlatformSurfaceCreateInfo VkXlibSurfaceCreateInfoKHR
#define PLATFORM_SURFACE_CREATE_INFO VK_STRUCTURE_TYPE_XLIB_SURFACE_CREATE_INFO_KHR;
#define PLATFORM_ASSIGN_SURFACEINFO \
    surfaceCreateInfo.dpy = info.info.x11.display; \
    surfaceCreateInfo.window = info.info.x11.window
#define PlatformCreateSurface vkCreateXlibSurfaceKHR
#endif /* _WIN32 */

//! Destructor

irr::video::CSDLVulkanPlatform::~CSDLVulkanPlatform()
{
}

void irr::video::CSDLVulkanPlatform::initialize(IrrlichtDevice* device)
{
    SDL_Window* window = static_cast<CIrrDeviceSDL*>(device)->getSDLWindow();
    SDL_SysWMinfo info;
    SDL_VERSION(&info.version);
    SDL_GetWindowWMInfo(window, &info);

    // Create Vulkan surface
    PlatformSurfaceCreateInfo surfaceCreateInfo;
    surfaceCreateInfo.sType = PLATFORM_SURFACE_CREATE_INFO;
    surfaceCreateInfo.pNext = nullptr;
    surfaceCreateInfo.flags = 0;
    PLATFORM_ASSIGN_SURFACEINFO;

    VkInstance instance = mDriver->_getInstance();
    VkResult result = PlatformCreateSurface(instance, &surfaceCreateInfo, VulkanDevice::gVulkanAllocator, &mSurface);
    assert(result == VK_SUCCESS);

    VulkanDevice* presentDevice = mDriver->_getPrimaryDevice();
    VkPhysicalDevice physicalDevice = presentDevice->getPhysical();

    mPresentQueueFamily = presentDevice->getQueueFamily(GQT_GRAPHICS);


    VkSurfaceCapabilitiesKHR cap;
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice, mSurface, &cap);

    VkBool32 supportsPresent;
    vkGetPhysicalDeviceSurfaceSupportKHR(physicalDevice, mPresentQueueFamily, mSurface, &supportsPresent);

    if (!supportsPresent)
    {
        // Note: Not supporting present only queues at the moment
        // Note: Also present device can only return one family of graphics queue, while there could be more (some of
        // which support present)
        throw std::runtime_error("Cannot find a graphics queue that also supports present operations.");
    }

    SurfaceFormat format = presentDevice->getSurfaceFormat(mSurface, mDriver->mCreateParams.Stencilbuffer, false);
    mColorFormat = format.colorFormat;
    mColorSpace = format.colorSpace;
    mDepthFormat = format.depthFormat;

    // Make the window full screen if required
    if (mDriver->mCreateParams.Fullscreen)
    {
        if (!mIsFullScreen)
        {
            // FIXME(manh): fix device mode for SDL2 on windows
            /*
            DEVMODE displayDeviceMode;

            memset(&displayDeviceMode, 0, sizeof(displayDeviceMode));
            displayDeviceMode.dmSize = sizeof(DEVMODE);
            displayDeviceMode.dmBitsPerPel = 32;
            displayDeviceMode.dmPelsWidth = mDriver->getScreenSize().Width;
            displayDeviceMode.dmPelsHeight = mDriver->getScreenSize().Height;
            displayDeviceMode.dmFields = DM_BITSPERPEL | DM_PELSWIDTH | DM_PELSHEIGHT;

            //if (mDisplayFrequency)
            //{
            //    displayDeviceMode.dmDisplayFrequency = mDisplayFrequency;
            //    displayDeviceMode.dmFields |= DM_DISPLAYFREQUENCY;
            //
            //    if (ChangeDisplaySettingsEx(NULL, &displayDeviceMode, NULL, CDS_FULLSCREEN | CDS_TEST, NULL) != DISP_CHANGE_SUCCESSFUL)
            //    {
            //        assert("ChangeDisplaySettings with user display frequency failed.");
            //    }
            //}

            if (ChangeDisplaySettingsEx(NULL, &displayDeviceMode, NULL, CDS_FULLSCREEN, NULL) != DISP_CHANGE_SUCCESSFUL)
            {
                assert("ChangeDisplaySettings failed.");
            }
            */
        }
    }

    // Create swap chain
    mSwapChain = new VulkanSwapChain();
    mSwapChain->rebuild(presentDevice, mSurface, mDriver->getScreenSize().Width, mDriver->getScreenSize().Height, mDriver->mCreateParams.Vsync, mColorFormat, mColorSpace, true, mDepthFormat);
}

void irr::video::CSDLVulkanPlatform::resizeSwapBuffers()
{
    /*
    // Resize swap chain

    //// Need to make sure nothing is using the swap buffer before we re-create it
    // Note: Optionally I can detect exactly on which queues (if any) are the swap chain images used on, and only wait
    // on those

    // Make the window full screen if required
    if (mDriver->mCreateParams.Fullscreen)
    {
        if (!mIsFullScreen)
        {
            DEVMODE displayDeviceMode;

            memset(&displayDeviceMode, 0, sizeof(displayDeviceMode));
            displayDeviceMode.dmSize = sizeof(DEVMODE);
            displayDeviceMode.dmBitsPerPel = 32;
            displayDeviceMode.dmPelsWidth = mDriver->getScreenSize().Width;
            displayDeviceMode.dmPelsHeight = mDriver->getScreenSize().Height;
            displayDeviceMode.dmFields = DM_BITSPERPEL | DM_PELSWIDTH | DM_PELSHEIGHT;

            //if (mDisplayFrequency)
            //{
            //    displayDeviceMode.dmDisplayFrequency = mDisplayFrequency;
            //    displayDeviceMode.dmFields |= DM_DISPLAYFREQUENCY;
            //
            //    if (ChangeDisplaySettingsEx(NULL, &displayDeviceMode, NULL, CDS_FULLSCREEN | CDS_TEST, NULL) != DISP_CHANGE_SUCCESSFUL)
            //    {
            //        assert("ChangeDisplaySettings with user display frequency failed.");
            //    }
            //}

            if (ChangeDisplaySettingsEx(NULL, &displayDeviceMode, NULL, CDS_FULLSCREEN, NULL) != DISP_CHANGE_SUCCESSFUL)
            {
                assert("ChangeDisplaySettings failed.");
            }
        }
    }

    */
    mDriver->_getPrimaryDevice()->waitIdle();
    mSwapChain->rebuild(mDriver->_getPrimaryDevice(), mSurface, mDriver->getScreenSize().Width, mDriver->getScreenSize().Height, mDriver->mCreateParams.Vsync, mColorFormat, mColorSpace, true, mDepthFormat);
}

#endif // _IRR_COMPILE_WITH_SDL_DEVICE_

