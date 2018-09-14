#ifndef __C_WGL_MANAGER_H_INCLUDED__
#define __C_WGL_MANAGER_H_INCLUDED__

#include "IrrCompileConfig.h"

#ifdef _IRR_COMPILE_WITH_SDL_DEVICE_

#include "CVulkanPlatform.h"

namespace irr
{
    class IrrlichtDevice;
namespace video
{
    // WGL manager.
    class CSDLVulkanPlatform : public CVulkanPlatform
    {
    public:
        //! Constructor.
        CSDLVulkanPlatform(CVulkanDriver* driver)
            : CVulkanPlatform(driver)
        {}

		//! Destructor
        virtual ~CSDLVulkanPlatform();

        // Inherited via CVulkanPlatform
        void initialize(IrrlichtDevice* device);
        virtual void initialize() override { }
        virtual void resizeSwapBuffers() override;

    protected:

        bool mIsFullScreen = false;

    };
}
}

#endif // _IRR_COMPILE_WITH_SDL_DEVICE_

#endif
