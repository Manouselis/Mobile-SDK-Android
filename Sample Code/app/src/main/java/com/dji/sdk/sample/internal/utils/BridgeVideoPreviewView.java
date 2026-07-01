package com.dji.sdk.sample.internal.utils;

import android.content.Context;
import android.graphics.SurfaceTexture;
import android.util.AttributeSet;
import android.view.TextureView;

import com.dji.sdk.sample.internal.view.MainContent;

import dji.midware.usb.P3.UsbAccessoryService;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;

/**
 * Tiny DJI preview activator used by the ROS video bridge.
 */
public class BridgeVideoPreviewView extends TextureView implements TextureView.SurfaceTextureListener {

    private DJICodecManager codecManager;
    private VideoFeeder.VideoDataListener videoDataListener;
    private VideoFeeder.VideoFeed registeredVideoFeed;
    private SurfaceTexture surfaceTexture;
    private int surfaceWidth = 1;
    private int surfaceHeight = 1;
    private boolean isPrimaryVideoFeed = true;

    public BridgeVideoPreviewView(Context context) {
        this(context, null, 0);
    }

    public BridgeVideoPreviewView(Context context, AttributeSet attrs) {
        this(context, attrs, 0);
    }

    public BridgeVideoPreviewView(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
        init();
    }

    private void init() {
        if (isInEditMode()) {
            return;
        }

        setOpaque(false);
        setSurfaceTextureListener(this);
        videoDataListener = new VideoFeeder.VideoDataListener() {
            @Override
            public void onReceive(byte[] videoBuffer, int size) {
                MainContent.forwardPreviewVideoPacket(
                        isPrimaryVideoFeed ? "preview-primary" : "preview-secondary",
                        videoBuffer,
                        size
                );

                if (codecManager != null) {
                    codecManager.sendDataToDecoder(
                            videoBuffer,
                            size,
                            isPrimaryVideoFeed
                                    ? UsbAccessoryService.VideoStreamSource.Camera.getIndex()
                                    : UsbAccessoryService.VideoStreamSource.Fpv.getIndex()
                    );
                }
            }
        };
    }

    public VideoFeeder.VideoDataListener registerLiveVideo(VideoFeeder.VideoFeed videoFeed, boolean isPrimary) {
        boolean sourceChanged = isPrimaryVideoFeed != isPrimary;
        isPrimaryVideoFeed = isPrimary;

        if (sourceChanged) {
            destroyCodecManager();
            createCodecManager();
        }

        if (registeredVideoFeed != null && registeredVideoFeed != videoFeed) {
            unregisterLiveVideo();
        }

        if (videoFeed != null && videoDataListener != null
                && !videoFeed.getListeners().contains(videoDataListener)) {
            videoFeed.addVideoDataListener(videoDataListener);
        }
        registeredVideoFeed = videoFeed;
        return videoDataListener;
    }

    public void unregisterLiveVideo() {
        if (registeredVideoFeed != null && videoDataListener != null) {
            try {
                registeredVideoFeed.removeVideoDataListener(videoDataListener);
            } catch (Exception ignored) {
            }
        }
        registeredVideoFeed = null;
    }

    public void changeSourceResetKeyFrame() {
        if (codecManager != null) {
            codecManager.resetKeyFrame();
        }
    }

    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        surfaceTexture = surface;
        surfaceWidth = Math.max(width, 1);
        surfaceHeight = Math.max(height, 1);
        createCodecManager();
    }

    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        surfaceWidth = Math.max(width, 1);
        surfaceHeight = Math.max(height, 1);
    }

    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        destroyCodecManager();
        surfaceTexture = null;
        return true;
    }

    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
    }

    @Override
    protected void onDetachedFromWindow() {
        unregisterLiveVideo();
        destroyCodecManager();
        super.onDetachedFromWindow();
    }

    private void createCodecManager() {
        if (codecManager != null || surfaceTexture == null) {
            return;
        }

        codecManager = new DJICodecManager(
                getContext().getApplicationContext(),
                surfaceTexture,
                surfaceWidth,
                surfaceHeight,
                isPrimaryVideoFeed
                        ? UsbAccessoryService.VideoStreamSource.Camera
                        : UsbAccessoryService.VideoStreamSource.Fpv
        );
    }

    private void destroyCodecManager() {
        if (codecManager != null) {
            codecManager.cleanSurface();
            codecManager.destroyCodec();
            codecManager = null;
        }
    }
}
