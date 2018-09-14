#pragma once

#include "CVulkanResources.h"
#include "CVulkanSharedDefines.h"
#include "CVulkanEventQuery.h"
#include "RenderEngines/General/CCommandBuffer.h"
#include "RenderEngines/General/CCommonDefines.h"

#include "SColor.h"

#include <standard/enum.h>
#include <unordered_set>
#include <vulkan/vulkan.h>

namespace irr
{
    namespace video
    {
        class VulkanOcclusionQuery;
        class VulkanTimerQuery;
        class CVulkanTexture;
        class VulkanImage;
        class VulkanFramebuffer;
        class VulkanBuffer;
        class CVulkanVertexDeclaration;
        class VulkanGraphicsPipelineState;
        class VulkanComputePipelineState;
        class VulkanSwapChain;
        class VulkanDevice;
        class VulkanGpuParams;
        class VulkanQueue;
        class VulkanPipeline;

        struct BufferInfo;
        struct ImageInfo;
        struct ImageSubresourceInfo;
        /** @addtogroup Vulkan
         *  @{
         */

#define _MAX_VULKAN_CB_PER_QUEUE_FAMILY _MAX_QUEUES_PER_TYPE * 32

         // Maximum number of command buffers that another command buffer can be dependant on (via a sync mask)
#define _MAX_VULKAN_CB_DEPENDENCIES 2

    /** Wrapper around a Vulkan semaphore object that manages its usage and lifetime. */
        class VulkanSemaphore : public CVulkanDeviceResource
        {
        public:
            VulkanSemaphore(CVulkanDriver* owner);
            virtual ~VulkanSemaphore();

            /** Returns the internal handle to the Vulkan object. */
            VkSemaphore getHandle() const { return mSemaphore; }

        private:
            VkSemaphore mSemaphore;

            // Inherited via CVulkanDeviceResource
            virtual void OnDeviceLost(irr::video::CVulkanDriver * device) _IRR_OVERRIDE_;
            virtual void OnDeviceRestored(irr::video::CVulkanDriver * device) _IRR_OVERRIDE_;
            virtual void OnDeviceDestroy(irr::video::CVulkanDriver* device) _IRR_OVERRIDE_ {}
        };

        class VulkanCmdBuffer;

        /** Pool that allocates and distributes Vulkan command buffers. */
        class VulkanCmdBufferPool
        {
        public:
            VulkanCmdBufferPool(irr::video::VulkanDevice& device);
            ~VulkanCmdBufferPool();

            /**
             * Attempts to find a free command buffer, or creates a new one if not found. Caller must guarantee the provided
             * queue family is valid.
             */
            VulkanCmdBuffer* getBuffer(uint32_t queueFamily, bool secondary);
            const irr::video::VulkanDevice* GetDevice() const { return &mDevice; }

        private:
            /** Command buffer pool and related information. */
            struct PoolInfo
            {
                VkCommandPool pool = VK_NULL_HANDLE;
                VulkanCmdBuffer* buffers[_MAX_VULKAN_CB_PER_QUEUE_FAMILY];
                uint32_t queueFamily = -1;
            };

            /** Creates a new command buffer. */
            VulkanCmdBuffer* createBuffer(uint32_t queueFamily, bool secondary);

            irr::video::VulkanDevice& mDevice;
            std::vector<PoolInfo> mPools;
            uint32_t mNextId;
        };

        /** Determines where are the current descriptor sets bound to. */
        enum class DescriptorSetBindFlag
        {
            Null = 0,
            Graphics = 1 << 0,
            Compute = 1 << 1
        };

        ENABLE_ENUM_CLASS_FLAG(irr::video::DescriptorSetBindFlag)

        /** Specifies for what purpose is a resource being bound to a command buffer. */
        enum class ResourceUsage
        {
            ShaderBind,
            Framebuffer,
            Transfer
        };

        /**
         * Represents a direct wrapper over an internal Vulkan command buffer. This is unlike VulkanCommandBuffer which is a
         * higher level class, and it allows for re-use by internally using multiple low-level command buffers.
         */
        class VulkanCmdBuffer : public irr::IReferenceCounted
        {
            /** Possible states a command buffer can be in. */
            enum class State
            {
                /** Buffer is ready to be re-used. */
                Ready,
                /** Buffer is currently recording commands, but isn't recording a render pass. */
                Recording,
                /** Buffer is currently recording render pass commands. */
                RecordingRenderPass,
                /** Buffer is done recording but hasn't been submitted. */
                RecordingDone,
                /** Buffer is done recording and is currently submitted on a queue. */
                Submitted
            };

        public:
            VulkanCmdBuffer(irr::video::VulkanDevice& device, uint32_t id, VkCommandPool pool, uint32_t queueFamily, bool secondary);
            virtual ~VulkanCmdBuffer();

            /** Returns an unique identifier of this command buffer. */
            uint32_t getId() const { return mId; }

            /** Returns the index of the queue family this command buffer is executing on. */
            uint32_t getQueueFamily() const { return mQueueFamily; }

            /** Returns the index of the device this command buffer will execute on. */
            uint32_t getDeviceIdx() const;

            /** Makes the command buffer ready to start recording commands. */
            void begin();

            /** Ends command buffer command recording (as started with begin()). */
            void end();

            /** Begins render pass recording. Must be called within begin()/end() calls. */
            void beginRenderPass();

            /** Ends render pass recording (as started with beginRenderPass(). */
            void endRenderPass();

            /**
             * Submits the command buffer for execution.
             *
             * @param[in]	queue		Queue to submit the command buffer on.
             * @param[in]	queueIdx	Index of the queue the command buffer was submitted on. Note that this may be different
             *							from the actual irr::video::VulkanQueue index since multiple command buffer queue indices can map
             *							to the same queue.
             * @param[in]	syncMask	Mask that controls which other command buffers does this command buffer depend upon
             *							(if any). See description of @p syncMask parameter in RenderAPI::executeCommands().
             */
            void submit(irr::video::VulkanQueue* queue, uint32_t queueIdx, uint32_t syncMask);

            /** Returns the handle to the internal Vulkan command buffer wrapped by this object. */
            VkCommandBuffer getHandle() const { return mCmdBuffer; }

            /** Returns a fence that can be used for tracking when the command buffer is done executing. */
            VkFence getFence() const { return mFence; }

            /**
             * Returns a semaphore that may be used for synchronizing execution between command buffers executing on the same
             * queue.
             */
            irr::video::VulkanSemaphore* getIntraQueueSemaphore() const { return mIntraQueueSemaphore; }

            /**
             * Returns a semaphore that may be used for synchronizing execution between command buffers executing on different
             * queues. Note that these semaphores get used each time they are requested, and there is only a fixed number
             * available. If all are used up, null will be returned. New semaphores are generated when allocateSemaphores()
             * is called.
             */
            irr::video::VulkanSemaphore* requestInterQueueSemaphore() const;

            /**
             * Allocates a new set of semaphores that may be used for synchronizing execution between different command buffers.
             * Releases the previously allocated semaphores, if they exist. Use getIntraQueueSemaphore() &
             * requestInterQueueSemaphore() to retrieve latest allocated semaphores.
             *
             * @param[out]	semaphores	Output array to place all allocated semaphores in. The array must be of size
             *							(_MAX_VULKAN_CB_DEPENDENCIES + 1).
             */
            void allocateSemaphores(VkSemaphore* semaphores);

            /** Returns true if the command buffer is currently being processed by the device. */
            bool isSubmitted() const { return mState == State::Submitted; }

            /** Returns true if the command buffer is currently recording (but not within a render pass). */
            bool isRecording() const { return mState == State::Recording; }

            /** Returns true if the command buffer is ready to be submitted to a queue. */
            bool isReadyForSubmit() const { return mState == State::RecordingDone; }

            /** Returns true if the command buffer is currently recording a render pass. */
            bool isInRenderPass() const { return mState == State::RecordingRenderPass; }

            /**
             * Checks the internal fence if done executing.
             *
             * @param[in]	block	If true, the system will block until the fence is signaled.
             */
            bool checkFenceStatus(bool block) const;

            /**
             * Resets the command buffer back in Ready state. Should be called when command buffer is done executing on a
             * queue.
             */
            void reset();

            /**
             * Lets the command buffer know that the provided resource has been queued on it, and will be used by the
             * device when the command buffer is submitted. If a resource is an image or a buffer use the more specific
             * registerResource() overload.
             */
            void registerResource(irr::video::CVulkanDeviceResource* res, VulkanUseFlag flags);

            /**
             * Lets the command buffer know that the provided image resource has been queued on it, and will be used by the
             * device when the command buffer is submitted. Executes a layout transition to @p newLayout (if needed), and
             * updates the externally visible image layout field to @p finalLayout (once submitted).
             *
             * @param[in]	res						Image to register with the command buffer.
             * @param[in]	range					Range of sub-resources to register.
             * @param[in]	newLayout				Layout the image needs to be transitioned in before use. Set to undefined
             *										layout if no transition is required.
             * @param[in]	finalLayout				Determines what value the externally visible image layout will be set after
             *										submit() is called. Normally this will be same as @p newLayout, but can be
             *										different if some form of automatic layout transitions are happening.
             * @param[in]	flags					Flags that determine how will be command buffer be using the buffer.
             * @param[in]	usage					Determines for what purpose is the resource being registered for.
             */
            void registerResource(irr::video::VulkanImage* res, const VkImageSubresourceRange& range, VkImageLayout newLayout,
                VkImageLayout finalLayout, VulkanUseFlag flags, ResourceUsage usage);

            /**
             * Lets the command buffer know that the provided image resource has been queued on it, and will be used by the
             * device when the command buffer is submitted. Assumes the image is in its optimal layout.
             */
            void registerResource(irr::video::VulkanImage* res, const VkImageSubresourceRange& range, VulkanUseFlag flags,
                ResourceUsage usage);

            /**
             * Lets the command buffer know that the provided image resource has been queued on it, and will be used by the
             * device when the command buffer is submitted.
             */
            void registerResource(irr::video::VulkanBuffer* res, VkAccessFlags accessFlags, VulkanUseFlag flags);

            /**
             * Lets the command buffer know that the provided framebuffer resource has been queued on it, and will be used by
             * the device when the command buffer is submitted.
             */
            void registerResource(irr::video::VulkanFramebuffer* res, RenderSurfaceMaskBits loadMask, uint32_t readMask);

            /** Notifies the command buffer that the provided query has been queued on it. */
            void registerQuery(irr::video::VulkanOcclusionQuery* query) { mOcclusionQueries.insert(query); }

            /** Notifies the command buffer that the provided query has been queued on it. */
            void registerQuery(irr::video::VulkanTimerQuery* query) { mTimerQueries.insert(query); }

            /************************************************************************/
            /* 								COMMANDS	                     		*/
            /************************************************************************/

            /**
             * Assigns a render target the the command buffer. This render target's framebuffer and render pass will be used
             * when beginRenderPass() is called. Command buffer must not be currently recording a render pass.
             */
            void setRenderTarget(const video::IRenderTarget* rt, uint32_t readOnlyFlags, RenderSurfaceMaskBits loadMask, bool LifeTimeBound);

            /** Clears the entirety currently bound render target. */
            void clearRenderTarget(uint32_t buffers, const irr::video::SColor& color, float depth, std::uint16_t stencil, std::uint8_t targetMask);

            /** Clears the viewport portion of the currently bound render target. */
            void clearViewport(uint32_t buffers, const irr::video::SColor& color, float depth, std::uint16_t stencil, std::uint8_t targetMask);

            /** Assigns a pipeline state to use for subsequent draw commands. */
            void setPipelineState(const irr::video::VulkanGraphicsPipelineState* state);

            /** Assigns a pipeline state to use for subsequent dispatch commands. */
            void setPipelineState(const irr::video::VulkanComputePipelineState* state);

            /** Assign GPU params to the GPU programs bound by the pipeline state. */
            void setGpuParams(irr::video::VulkanGpuParams* gpuParams);

            /** Sets the current viewport which determine to which portion of the render target to render to. */
            void setViewport(const irr::core::recti& area);

            /**
             * Sets the scissor rectangle area which determines in which area if the viewport are the fragments allowed to be
             * generated. Only relevant if enabled on the pipeline state.
             */
            void setScissorRect(const irr::core::recti& area);

            /** Sets a stencil reference value that will be used for comparisons in stencil operations, if enabled. */
            void setStencilRef(uint32_t value);

            /** Changes how are primitives interpreted as during rendering. */
            void setDrawOp(scene::E_PRIMITIVE_TYPE drawOp);

            /** Sets one or multiple vertex buffers that will be used for subsequent draw() or drawIndexed() calls. */
            void setVertexBuffers(uint32_t index, irr::video::VulkanBuffer** buffers, uint32_t numBuffers, VkDeviceSize* offset);

            /** Sets an index buffer that will be used for subsequent drawIndexed() calls. */
            void setIndexBuffer(irr::video::VulkanBuffer* buffer, E_INDEX_TYPE type, VkDeviceSize offset);

            /** Sets a declaration that determines how are vertex buffer contents interpreted. */
            void setVertexDeclaration(const irr::video::CVulkanVertexDeclaration* decl);

            /** Executes a draw command using the currently bound graphics pipeline, vertex buffer and render target. */
            void draw(uint32_t vertexOffset, uint32_t vertexCount, uint32_t instanceCount);

            /** Executes a draw command using the currently bound graphics pipeline, index & vertex buffer and render target. */
            void drawIndexed(uint32_t startIndex, uint32_t indexCount, uint32_t vertexOffset, uint32_t instanceCount);

            /** Executes a dispatch command using the currently bound compute pipeline. */
            void dispatch(uint32_t numGroupsX, uint32_t numGroupsY, uint32_t numGroupsZ);

            /**
             * Registers a command that signals the event when executed. Will be delayed until the end of the current
             * render pass, if any.
             */
            void setEvent(irr::video::VulkanEvent* event);

            /**
             * Registers a command that resets the query. The command will be delayed until the next submit() if a render
             * pass is currently in progress, but is guaranteed to execute before this command buffer is submitted.
             */
            void resetQuery(irr::video::VulkanQuery* query);

            /**
             * Issues a pipeline barrier on the provided buffer. See vkCmdPipelineBarrier in Vulkan spec. for usage
             * information.
             */
            void memoryBarrier(VkBuffer buffer, VkAccessFlags srcAccessFlags, VkAccessFlags dstAccessFlags,
                VkPipelineStageFlags srcStage, VkPipelineStageFlags dstStage);

            /**
             * Issues a pipeline barrier on the provided image. See vkCmdPipelineBarrier in Vulkan spec. for usage
             * information.
             */
            void memoryBarrier(VkImage image, VkAccessFlags srcAccessFlags, VkAccessFlags dstAccessFlags,
                VkPipelineStageFlags srcStage, VkPipelineStageFlags dstStage, VkImageLayout oldlayout, VkImageLayout layout,
                const VkImageSubresourceRange& range);

            /**
             * Issues a pipeline barrier on the provided image, changing its layout. See vkCmdPipelineBarrier in Vulkan spec.
             * for usage information.
             */
            void setLayout(VkImage image, VkAccessFlags srcAccessFlags, VkAccessFlags dstAccessFlags,
                VkImageLayout oldLayout, VkImageLayout newLayout, const VkImageSubresourceRange& range);

            /**
             * Returns the current layout of the specified image, as seen by this command buffer. This is different from the
             * global layout stored in CVulkanTexture itself, as it includes any transitions performed by the command buffer
             * (at the current point in time), while the global layout is only updated after a command buffer as been submitted.
             *
             * @param[in]	image			Image to lookup the layout for.
             * @param[in]	range			Subresource range of the image to lookup the layout for.
             * @param[in]	inRenderPass	If true this will return the layout of the image after the render pass begins.
             *								If false it will return the current layout of the image. These may be different
             *								in the case the image is used in the framebuffer, in which case the render pass
             *								may perform an automated layout transition when it begins.
             */
            VkImageLayout getCurrentLayout(irr::video::VulkanImage* image, const VkImageSubresourceRange& range, bool inRenderPass);

            irr::video::VulkanGpuParams* getGpuParams() { return mBoundParams; }

            void setSwapChain(irr::video::VulkanSwapChain* chain) { mSwapChains.push_back(chain); }

        private:
            friend class VulkanCmdBufferPool;
            friend class VulkanCommandBuffer;
            friend class irr::video::VulkanQueue;

            /** Checks if all the prerequisites for rendering have been made (e.g. render target and pipeline state are set.) */
            bool isReadyForRender();

            /** Marks the command buffer as submitted on a queue. */
            void setIsSubmitted() { mState = State::Submitted; }

            /** Binds the current graphics pipeline to the command buffer. Returns true if bind was successful. */
            bool bindGraphicsPipeline();

            /**
             * Binds any dynamic states to the pipeline, as required.
             *
             * @param[in]	forceAll	If true all states will be bound. If false only states marked as dirty will be bound.
             */
            void bindDynamicStates(bool forceAll);

            /** Binds the currently stored GPU parameters object, if dirty. */
            void bindGpuParams();

            /** Clears the specified area of the currently bound render target. */
            void clearViewport(const core::recti& area, uint32_t buffers, const irr::video::SColor& color, float depth, std::uint16_t stencil, std::uint8_t targetMask);

            /** Starts and ends a render pass, intended only for a clear operation. */
            void executeClearPass();

            /** Executes any queued layout transitions by issuing a pipeline barrier. */
            void executeLayoutTransitions();

            /**
             * Updates final layouts for images used by the current framebuffer, reflecting layout changes performed by render
             * pass' automatic layout transitions.
             */
            void updateFinalLayouts();

            /**
             * Updates an existing sub-resource info range with new layout, use flags and framebuffer flag. Returns true if
             * the bound sub-resource is a read-only framebuffer attachment.
             */
            bool updateSubresourceInfo(irr::video::VulkanImage* image, uint32_t imageInfoIdx, ImageSubresourceInfo& subresourceInfo,
                VkImageLayout newLayout, VkImageLayout finalLayout, VulkanUseFlag flags, ResourceUsage usage);

            /** Finds a subresource info structure containing the specified face and mip level of the provided image. */
            ImageSubresourceInfo& findSubresourceInfo(irr::video::VulkanImage* image, uint32_t face, uint32_t mip);

            /** Gets all queries registered on this command buffer that haven't been ended. */
            void getInProgressQueries(std::vector<irr::video::VulkanTimerQuery*>& timer, std::vector<irr::video::VulkanOcclusionQuery*>& occlusion) const;

            /** Returns the read mask for the current framebuffer. */
            RenderSurfaceMaskBits getFBReadMask();

            uint32_t mId;
            uint32_t mQueueFamily;
            State mState;
            irr::video::VulkanDevice& mDevice;
            VkCommandPool mPool;
            VkCommandBuffer mCmdBuffer;
            VkFence mFence;

            irr::video::VulkanSemaphore* mIntraQueueSemaphore;
            irr::video::VulkanSemaphore* mInterQueueSemaphores[_MAX_VULKAN_CB_DEPENDENCIES];
            mutable uint32_t mNumUsedInterQueueSemaphores;

            irr::video::VulkanPipeline* mPipeline;
            irr::video::VulkanFramebuffer* mFramebuffer;
            uint32_t mRenderTargetWidth;
            uint32_t mRenderTargetHeight;
            uint32_t mRenderTargetReadOnlyFlags;
            RenderSurfaceMaskBits mRenderTargetLoadMask;

            std::vector<irr::video::CVulkanDeviceResource*> mResources;
            std::vector<irr::video::CVulkanDeviceResource*> mImages;
            std::vector<irr::video::VulkanBuffer*> mBuffers;
            std::set<irr::video::VulkanOcclusionQuery*> mOcclusionQueries;
            std::set<irr::video::VulkanTimerQuery*> mTimerQueries;
            //std::vector<ImageInfo> mImageInfos;
            //std::vector<ImageSubresourceInfo> mSubresourceInfoStorage;
            //std::set<uint32_t> mPassTouchedSubresourceInfos; // All subresource infos touched by the current render pass
            uint32_t mGlobalQueueIdx;

            irr::video::VulkanGraphicsPipelineState* mGraphicsPipeline;
            irr::video::VulkanComputePipelineState* mComputePipeline;
            irr::video::CVulkanVertexDeclaration* mVertexDecl;
            irr::core::recti mViewport;
            irr::core::recti mScissor;
            uint32_t mStencilRef;
            irr::scene::E_PRIMITIVE_TYPE mDrawOp;
            uint32_t mNumBoundDescriptorSets;
            bool mGfxPipelineRequiresBind : 1;
            bool mCmpPipelineRequiresBind : 1;
            bool mViewportRequiresBind : 1;
            bool mStencilRefRequiresBind : 1;
            bool mScissorRequiresBind : 1;
            bool mBoundParamsDirty : 1;
            bool mPermanentFrameBuffer : 1;
            DescriptorSetBindFlag mDescriptorSetsBindState;
            irr::video::VulkanGpuParams* mBoundParams;

            std::array<VkClearValue, _MAX_MULTIPLE_RENDER_TARGETS + 1> mClearValues;
            uint32_t mClearMask;
            irr::core::recti mClearArea;

            std::vector<irr::video::VulkanSemaphore*> mSemaphoresTemp;
            VkViewport viewport;
            VkRect2D scissorRect;
            VkBuffer mVertexBuffersTemp[_MAX_BOUND_VERTEX_BUFFERS];
            VkDeviceSize mVertexBufferOffsetsTemp[_MAX_BOUND_VERTEX_BUFFERS];
            VkDescriptorSet* mDescriptorSetsTemp;
            std::map<uint32_t, TransitionInfo> mTransitionInfoTemp;
            std::vector<VkImageMemoryBarrier> mLayoutTransitionBarriersTemp;
            std::vector<irr::video::VulkanImage*> mQueuedLayoutTransitions;
            std::vector<irr::video::VulkanEvent*> mQueuedEvents;
            std::vector<irr::video::VulkanQuery*> mQueuedQueryResets;
            std::vector < irr::video::VulkanSwapChain* > mSwapChains;
        };

        /** CommandBuffer implementation for Vulkan. */
        class VulkanCommandBuffer : public CommandBuffer
        {
        public:
            VulkanCommandBuffer(irr::video::CVulkanDriver* device, irr::video::GpuQueueType type, uint32_t deviceIdx, uint32_t queueIdx, bool secondary);
            virtual ~VulkanCommandBuffer();

            /**
             * Submits the command buffer for execution.
             *
             * @param[in]	syncMask	Mask that controls which other command buffers does this command buffer depend upon
             *							(if any). See description of @p syncMask parameter in RenderAPI::executeCommands().
             */
            void submit(uint32_t syncMask);

            /**
             * Returns the internal command buffer.
             *
             * @note	This buffer will change after a submit() call.
             */
            VulkanCmdBuffer* getInternal() const { return mBuffer; }
            irr::video::CVulkanDriver* GetDriver() { return &mDevice; }

        private:
            /**
             * Tasks the command buffer to find a new internal command buffer. Call this after the command buffer has been
             * submitted to a queue (it's not allowed to be used until the queue is done with it).
             */
            void acquireNewBuffer();

            VulkanCmdBuffer* mBuffer;
            irr::video::CVulkanDriver& mDevice;
            irr::video::VulkanQueue* mQueue;
            uint32_t mIdMask;

            std::vector<irr::video::VulkanTimerQuery*> mcacheTimerQueries;
            std::vector<irr::video::VulkanOcclusionQuery*> mcacheOcclusionQueries;
        };

        /** Wrapper around a command buffer used specifically for transfer operations. */
        class VulkanTransferBuffer
        {
        public:
            VulkanTransferBuffer();
            VulkanTransferBuffer(irr::video::VulkanDevice* device, irr::video::GpuQueueType type, u32 queueIdx);
            ~VulkanTransferBuffer();

            /**
            * OR's the provided sync mask with the internal sync mask. The sync mask determines on which queues should
            * the buffer wait on before executing. Sync mask is reset after a flush. See CommandSyncMask on how to generate
            * a sync mask.
            */
            void appendMask(u32 syncMask) { mSyncMask |= syncMask; }

            /** Resets the sync mask. */
            void clearMask() { mSyncMask = 0; }

            /**
            * Issues a pipeline barrier on the provided buffer. See vkCmdPipelineBarrier in Vulkan spec. for usage
            * information.
            */
            void memoryBarrier(VkBuffer buffer, VkAccessFlags srcAccessFlags, VkAccessFlags dstAccessFlags,
                VkPipelineStageFlags srcStage, VkPipelineStageFlags dstStage);

            /**
            * Issues a pipeline barrier on the provided image, changing its layout. See vkCmdPipelineBarrier in Vulkan spec.
            * for usage information.
            */
            void setLayout(VkImage image, VkAccessFlags srcAccessFlags, VkAccessFlags dstAccessFlags,
                VkImageLayout oldLayout, VkImageLayout newLayout, const VkImageSubresourceRange& range);

            /**
            * Issues one or multiple pipeline barrier on the provided image, changing the layout of its subresources.
            * Automatically determines original layout for individual sub-resources, groups the pipeline barriers and issues
            * them.
            */
            void setLayout(irr::video::VulkanImage* image, const VkImageSubresourceRange& range, VkAccessFlags newAccessMask,
                VkImageLayout newLayout);

            /**
            * Submits the command buffer on the queue.
            *
            *	@param[in]	wait	If true, the caller thread will wait until all device operations on the command buffer's
            *						queue complete.
            */
            void flush(bool wait);

            /** Returns the internal command buffer. */
            VulkanCmdBuffer* getCB() const { return mCB; }
        private:
            friend class irr::video::VulkanDevice;

            /** Allocates a new internal command buffer. */
            void allocate();

            irr::video::VulkanDevice* mDevice;
            irr::video::GpuQueueType mType;
            u32 mQueueIdx;
            irr::video::VulkanQueue* mQueue;
            u32 mQueueMask;

            VulkanCmdBuffer* mCB;
            u32 mSyncMask;

            std::vector<VkImageMemoryBarrier> mBarriersTemp;
        };
        /** @} */
    }
}