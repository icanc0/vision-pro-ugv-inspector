//=============================================================================
// Copyright (c) 2001-2023 FLIR Systems, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================

#ifndef __LADYBUGVIDEO_H__
#define __LADYBUGVIDEO_H__

/** 
 * @defgroup LadybugVideo_h ladybugVideo.h
 *
 *  ladybugvideo.h
 *
 *  We welcome your bug reports, suggestions, and comments: 
 *  www.ptgrey.com/support/contact
 */

/*@{*/

#ifdef _WIN32

#ifdef LADYBUGVIDEO_EXPORTS
#define LADYBUGVIDEO_API __declspec(dllexport)
#elif defined(LADYBUGVIDEO_STATIC)
#define LADYBUGVIDEO_API
#else
#define LADYBUGVIDEO_API __declspec(dllimport)
#endif

#else

#if __GNUC__ >= 4
#ifdef LADYBUGVIDEO_EXPORTS
#define LADYBUGVIDEO_API __attribute__((visibility("default")))
#else
#define LADYBUGVIDEO_API __attribute__((visibility("default")))
#endif
#else
#define LADYBUGVIDEO_API
#endif

#endif

#ifdef __cplusplus
extern "C"
{
#endif

#include "ladybug.h"

/**
 * The Ladybug video context. To access video-specific methods, a  
 * LadybugVideoContext must be created using ladybugCreateVideoContext().
 * Once created, the context is passed to the particular function.
 */
typedef void* LadybugVideoContext;

/** Options for saving H.264 files. */
typedef struct LadybugH264Option
{
   /** Frame rate of the stream */        
   float frameRate;

   /** Width of source image */        
   unsigned int width;
        
   /** Height of source image */
   unsigned int height;

   /** Bitrate to encode at */
   unsigned int bitrate;

   /** Reserved for future use */
   unsigned int reserved[256];

} LadybugH264Option;

/** 
 * @defgroup VideoContextCreationMethods Video Context Creation and Initialization Methods
 *
 * This group of functions provides control over video functionality.
 */

/*@{*/ 

/**
 * Creates a new context for accessing video-specific functions of the
 * library.  
 *
 * A video context is useful for creating a video clip of the stitched image.
 * Currently, only H.264 codec is supported for creating a video.
 *
 * @param pContext - A pointer to a LadybugVideoContext to fill with the created context.
 *
 * @return A LadybugError indicating the success of the function.
 *
 * @see ladybugDestroyVideoContext()
 */
LADYBUGVIDEO_API LadybugError ladybugCreateVideoContext(LadybugVideoContext* pContext);

/**
 * Frees memory associated with the LadybugVideoContext.
 *
 * @param pContext - A pointer to the LadybugVideoContext to destroy.
 *
 * @return A LadybugError indicating the success of the function.
 *
 * @see ladybugCreateVideoContext()
 */
LADYBUGVIDEO_API LadybugError ladybugDestroyVideoContext(LadybugVideoContext* pContext);

/**
 * Opens a video file. 
 * Once opened, you can append as many frames as you want to the video. 
 * After finishing appending frames, you must close the video.
 *
 * @param context     - The video context. This must be created beforehand.
 * @param pszFileName - The file path to save.
 * @param pOptions    - The options for the codec.
 *
 * @return A LadybugError indicating the success of the function.
 *
 * @see ladybugCreateVideoContext()
 * @see ladybugCloseVideo()
 * @see ladybugAppendVideoFrame()
 */
LADYBUGVIDEO_API LadybugError
ladybugOpenVideo( 
    LadybugVideoContext context,
    const char* pszFileName,
    LadybugH264Option* pOptions );

/**
 * Closes a video file.
 *
 * @param context - The video context.
 *
 * @return A LadybugError indicating the success of the function.
 *
 * @see ladybugCreateVideoContext()
 * @see ladybugOpenVideo()
 */
LADYBUGVIDEO_API LadybugError ladybugCloseVideo( 
    LadybugVideoContext context);

/**
 * Appends a frame to the video.
 * The video must be opened before appending frames.
 *
 * @param context - The video context.
 * @param pImage  - An image to be appended. This is created by ladybugRenderOffScreenImage.
 *
 * @return A LadybugError indicating the success of the function.
 *
 * @see ladybugCreateVideoContext()
 * @see ladybugOpenVideo()
 * @see ladybugRenderOffScreenImage()
 */
LADYBUGVIDEO_API LadybugError ladybugAppendVideoFrame( 
    LadybugVideoContext context,
    LadybugProcessedImage* pImage);

/*@}*/

/*@}*/

#ifdef __cplusplus
};
#endif

#endif // #ifndef __LADYBUGVIDEO_H__
