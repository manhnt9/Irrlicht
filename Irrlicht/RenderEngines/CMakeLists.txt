# Configure Render System
if (IRR_BUILD_RENDERSYSTEM_GL EQUAL 1)
    add_subdirectory(OpenGL)
endif(IRR_BUILD_RENDERSYSTEM_GL EQUAL 1)

set(IRR_BUILD_RENDERSYSTEM_GL 0 CACHE STRING "0")

IF (NOT ${CMAKE_CXX_PLATFORM_ID} MATCHES MinGW AND WIN32)
  if (DirectX9_INCLUDE_DIR AND DirectX9_D3DX9_LIBRARY)
      add_subdirectory(DirectX9)
      set(IRR_BUILD_RENDERSYSTEM_D3DX9 1 CACHE STRING "1" FORCE)
  else ()
      MESSAGE(${LibExSTATUS} "Could not find dependency: DirectX9")
      MESSAGE(${LibExSTATUS} "Skipping DirectX9 rendersystem build")
      MESSAGE(${LibExSTATUS} "DirectX9 rendersystem variables: ${DirectX9_INCLUDE_DIR} ${DirectX9_D3DX9_LIBRARY}")
      set(IRR_BUILD_RENDERSYSTEM_D3DX9 0 CACHE STRING "0" FORCE)
  endif ()
  
  if (DirectX11_INCLUDE_DIR AND (DirectX11_D3D11_LIBRARY OR DirectX11_D3DX11_LIBRARY))
      add_subdirectory(Direct3D11)
      set(IRR_BUILD_RENDERSYSTEM_D3D11 1 CACHE STRING "1" FORCE)
  else ()
      MESSAGE(${LibExSTATUS} "Could not find dependency: Direct3D11")
      MESSAGE(${LibExSTATUS} "Skipping Direct3D11 rendersystem build")
      MESSAGE(${LibExSTATUS} "Direct3D11 rendersystem variables: ${DirectX11_INCLUDE_DIR} ${DirectX11_D3D11_LIBRARY} ${DirectX11_D3DX11_LIBRARY}")
      set(IRR_BUILD_RENDERSYSTEM_D3D11 0 CACHE STRING "0" FORCE)
  endif ()
ENDIF()

if (Vulkan_FOUND)
    add_subdirectory(Vulkan)
    set(IRR_BUILD_RENDERSYSTEM_Vulkan 1 CACHE STRING "1" FORCE)
else ()
    MESSAGE(${LibExSTATUS} "Could not find dependency: Vulkan")
    MESSAGE(${LibExSTATUS} "Skipping Vulkan rendersystem build")
    set(IRR_BUILD_RENDERSYSTEM_Vulkan 0 CACHE STRING "0" FORCE)
endif ()
