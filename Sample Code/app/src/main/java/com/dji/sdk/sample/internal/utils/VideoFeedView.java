package com.dji.sdk.sample.internal.utils;

import android.content.Context;
import android.graphics.Matrix;
import android.util.AttributeSet;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;

import com.dji.sdk.sample.internal.view.MainContent;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

import androidx.annotation.NonNull;
import dji.midware.usb.P3.UsbAccessoryService;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;
import dji.thirdparty.rx.Observable;
import dji.thirdparty.rx.Subscription;
import dji.thirdparty.rx.android.schedulers.AndroidSchedulers;
import dji.thirdparty.rx.functions.Action1;

/**
 * VideoView will show the live video for the given video feed.
 */
public class VideoFeedView extends SurfaceView {
    //region Properties
    private final static String TAG = "DULFpvWidget";
    private DJICodecManager codecManager = null;
    private VideoFeeder.VideoDataListener videoDataListener = null;
    private VideoFeeder.VideoFeed registeredVideoFeed = null;
    private int videoWidth;
    private int videoHeight;
    private boolean isPrimaryVideoFeed;
    private View coverView;
    private final long WAIT_TIME = 500; // Half of a second
    private AtomicLong lastReceivedFrameTime = new AtomicLong(0);
    private Observable timer =
        Observable.timer(100, TimeUnit.MICROSECONDS).observeOn(AndroidSchedulers.mainThread()).repeat();
    private Subscription subscription;
    private SurfaceHolder surfaceHolder;

    //endregion

    //region Life-Cycle
    public VideoFeedView(Context context) {
        this(context, null, 0);
    }

    public VideoFeedView(Context context, AttributeSet attrs) {
        this(context, attrs, 0);
    }

    public VideoFeedView(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
        init(context);
    }

    public void setCoverView(View view) {
        coverView = view;
    }

    private void init(Context context) {
        // Avoid the rending exception in the Android Studio Preview view.
        if (isInEditMode()) {
            return;
        }

        surfaceHolder = getHolder();
        surfaceHolder.addCallback(new SurfaceHolder.Callback() {
            @Override
            public void surfaceCreated(@NonNull SurfaceHolder surfaceHolder) {
                createCodecManager();
            }

            @Override
            public void surfaceChanged(@NonNull SurfaceHolder surfaceHolder, int format, int width, int height) {

            }

            @Override
            public void surfaceDestroyed(@NonNull SurfaceHolder surfaceHolder) {
                destroyCodecManager();
            }
        });

        videoDataListener = new VideoFeeder.VideoDataListener() {

            @Override
            public void onReceive(byte[] videoBuffer, int size) {

                lastReceivedFrameTime.set(System.currentTimeMillis());
                MainContent.forwardPreviewVideoPacket(
                        isPrimaryVideoFeed ? "preview-primary" : "preview-secondary",
                        videoBuffer,
                        size
                );

                if (codecManager != null) {
                    codecManager.sendDataToDecoder(videoBuffer,
                                                   size,
                                                   isPrimaryVideoFeed
                                                   ? UsbAccessoryService.VideoStreamSource.Camera.getIndex()
                                                   : UsbAccessoryService.VideoStreamSource.Fpv.getIndex());
                }
            }
        };

        subscription = timer.subscribe(new Action1() {
            @Override
            public void call(Object o) {
                final long now = System.currentTimeMillis();
                final long ellapsedTime = now - lastReceivedFrameTime.get();
                if (coverView != null) {
                    if (ellapsedTime > WAIT_TIME && !ModuleVerificationUtil.isMavic2Product()) {
                        if (coverView.getVisibility() == INVISIBLE) {
                            coverView.setVisibility(VISIBLE);
                        }
                    } else {
                        if (coverView.getVisibility() == VISIBLE) {
                            coverView.setVisibility(INVISIBLE);
                        }
                    }
                }
            }
        });
    }

    public VideoFeeder.VideoDataListener registerLiveVideo(VideoFeeder.VideoFeed videoFeed, boolean isPrimary) {
        boolean sourceChanged = isPrimaryVideoFeed != isPrimary;
        isPrimaryVideoFeed = isPrimary;

        if (sourceChanged && codecManager != null) {
            destroyCodecManager();
            createCodecManager();
        }

        if (videoDataListener != null && videoFeed != null && registeredVideoFeed != videoFeed) {
            unregisterLiveVideo();
        }

        if (videoDataListener != null && videoFeed != null && !videoFeed.getListeners().contains(videoDataListener)) {
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
    protected void onDetachedFromWindow() {
        super.onDetachedFromWindow();
        unregisterLiveVideo();
        destroyCodecManager();
        if (subscription != null && !subscription.isUnsubscribed()) {
            subscription.unsubscribe();
        }
    }

    private void createCodecManager() {
        if (codecManager != null || surfaceHolder == null || surfaceHolder.getSurface() == null
                || !surfaceHolder.getSurface().isValid()) {
            return;
        }

        codecManager = new DJICodecManager(
                getContext(),
                surfaceHolder,
                Math.max(getWidth(), 1),
                Math.max(getHeight(), 1),
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
