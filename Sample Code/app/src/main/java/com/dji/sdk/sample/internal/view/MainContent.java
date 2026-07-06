package com.dji.sdk.sample.internal.view;

import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageManager;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.os.Message;
import android.text.Editable;
import android.text.TextUtils;
import android.text.TextWatcher;
import android.util.AttributeSet;
import android.util.Log;
import android.view.KeyEvent;
import android.view.View;
import android.view.inputmethod.EditorInfo;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.ProgressBar;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;
import com.dji.sdk.sample.R;
import com.dji.sdk.sample.demo.bluetooth.BluetoothView;
import com.dji.sdk.sample.internal.controller.DJISampleApplication;
import com.dji.sdk.sample.internal.controller.MainActivity;
import com.dji.sdk.sample.internal.model.ViewWrapper;
import com.dji.sdk.sample.internal.utils.BridgeVideoPreviewView;
import com.dji.sdk.sample.internal.utils.DialogUtils;
import com.dji.sdk.sample.internal.utils.GeneralUtils;
import com.dji.sdk.sample.internal.utils.ToastUtils;
import com.squareup.otto.Subscribe;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import dji.common.error.DJIError;
import dji.common.error.DJISDKError;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.flightcontroller.RTKState;
import dji.common.gimbal.Attitude;
import dji.common.model.LocationCoordinate2D;
import dji.common.realname.AppActivationState;
import dji.common.useraccount.UserAccountState;
import dji.common.util.CommonCallbacks;
import dji.keysdk.DJIKey;
import dji.keysdk.FlightControllerKey;
import dji.keysdk.GimbalKey;
import dji.keysdk.KeyManager;
import dji.keysdk.ProductKey;
import dji.keysdk.callback.KeyListener;
import dji.log.DJILog;
import dji.log.GlobalConfig;
import dji.sdk.base.BaseComponent;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.products.Aircraft;
import dji.sdk.realname.AppActivationManager;
import dji.sdk.sdkmanager.BluetoothProductConnector;
import dji.sdk.sdkmanager.DJISDKInitEvent;
import dji.sdk.sdkmanager.DJISDKManager;
import dji.sdk.sdkmanager.LDMModule;
import dji.sdk.sdkmanager.LDMModuleType;
import dji.sdk.useraccount.UserAccountManager;

import java.io.BufferedOutputStream;
import java.io.DataOutputStream;
import java.net.InetSocketAddress;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InterfaceAddress;
import java.net.NetworkInterface;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

/**
 * Created by dji on 15/12/18.
 */
public class MainContent extends RelativeLayout {

    public static final String TAG = MainContent.class.getName();
    private static volatile MainContent activeVideoBridge;
    private String[] permissionArrays;
    private static final int REQUEST_PERMISSION_CODE = 12345;
    private AtomicBoolean isRegistrationInProgress = new AtomicBoolean(false);
    private int lastProcess = -1;
    private Handler mHander = new Handler();
    private static final int GIMBAL_UDP_PORT = 5005;
    private static final int VIDEO_TCP_PORT = 5006;
    private static final int DISCOVERY_UDP_PORT = 5007;
    private static final int POSITION_UDP_PORT = 5008;
    private static final int DISCOVERY_TIMEOUT_MS = 1500;
    private static final int DISCOVERY_ATTEMPTS = 3;
    private static final int DISCOVERY_RETRY_DELAY_MS = 3000;
    private static final String ROS_PC_IP_PREF_KEY = "dji_ros_pc_ip";
    private static final String DISCOVERY_REQUEST = "DJI_ROS2_BRIDGE_DISCOVER";
    private static final String DISCOVERY_RESPONSE_PREFIX = "DJI_ROS2_BRIDGE|";
    private static final int VIDEO_QUEUE_CAPACITY = 120;
    private static final int VIDEO_CONNECT_TIMEOUT_MS = 3000;

    private final ScheduledExecutorService udpExecutor = Executors.newSingleThreadScheduledExecutor();
    private final ExecutorService bridgeExecutor = Executors.newSingleThreadExecutor();
    private final ExecutorService videoExecutor = Executors.newSingleThreadExecutor();
    private final BlockingQueue<byte[]> videoQueue = new ArrayBlockingQueue<>(VIDEO_QUEUE_CAPACITY);
    private volatile DatagramSocket gimbalSocket;
    private volatile DatagramSocket positionSocket;
    private volatile InetAddress rosPcAddress;
    private volatile String manualRosPcIp = "";
    private volatile boolean bridgeStartupRunning = false;
    private int bridgeStartGeneration = 0;
    private volatile boolean gimbalBridgeStarted = false;
    private volatile boolean positionBridgeStarted = false;
    private DJIKey gimbalAttitudeKey;
    private KeyListener gimbalAttitudeUpdater;
    private ScheduledFuture<?> gimbalSendTask;
    private volatile Attitude latestGimbalAttitude;
    private DJIKey aircraftLocationKey;
    private DJIKey velocityXKey;
    private DJIKey velocityYKey;
    private DJIKey velocityZKey;
    private DJIKey satelliteCountKey;
    private DJIKey gpsSignalLevelKey;
    private DJIKey rtkStateKey;
    private KeyListener aircraftLocationUpdater;
    private KeyListener velocityXUpdater;
    private KeyListener velocityYUpdater;
    private KeyListener velocityZUpdater;
    private KeyListener satelliteCountUpdater;
    private KeyListener gpsSignalLevelUpdater;
    private KeyListener rtkStateUpdater;
    private ScheduledFuture<?> positionSendTask;
    private volatile LocationCoordinate3D latestAircraftLocation;
    private volatile Float latestVelocityX;
    private volatile Float latestVelocityY;
    private volatile Float latestVelocityZ;
    private volatile Integer latestSatelliteCount;
    private volatile Object latestGpsSignalLevel;
    private volatile RTKState latestRtkState;
    private VideoFeeder.VideoFeed primaryVideoFeed;
    private VideoFeeder.VideoFeed secondaryVideoFeed;
    private VideoFeeder.VideoFeed transcodedVideoFeed;
    private VideoFeeder.VideoDataListener primaryVideoDataListener;
    private VideoFeeder.VideoDataListener secondaryVideoDataListener;
    private VideoFeeder.VideoDataListener transcodedVideoDataListener;
    private VideoFeeder.VideoActiveStatusListener primaryVideoActiveStatusListener;
    private VideoFeeder.VideoActiveStatusListener secondaryVideoActiveStatusListener;
    private VideoFeeder.VideoActiveStatusListener transcodedVideoActiveStatusListener;
    private volatile String selectedVideoFeedName;
    private volatile Socket videoSocket;
    private volatile boolean videoBridgeStarted = false;
    private volatile boolean videoBridgeRunning = false;
    private BaseComponent.ComponentListener mDJIComponentListener = new BaseComponent.ComponentListener() {

        @Override
        public void onConnectivityChange(boolean isConnected) {
        Log.d(TAG, "onComponentConnectivityChanged: " + isConnected);
            notifyStatusChange();
        }
    };
    private ProgressBar progressBar;
    private static BluetoothProductConnector connector = null;
    private TextView mTextConnectionStatus;
    private TextView mTextProduct;
    private TextView mTextModelAvailable;
    private Button mBtnRegisterApp;
    private Button getmBtnRegisterAppForLDM;
    private Button mBtnOpen;
    private Button mBtnBluetooth;
    private BridgeVideoPreviewView mBridgePreviewFeed;
    private ViewWrapper componentList =
            new ViewWrapper(new DemoListView(getContext()), R.string.activity_component_list);
    private ViewWrapper bluetoothView;
    private EditText mBridgeModeEditText;
    private EditText mRosBridgeIpEditText;
    private CheckBox mCheckboxFirmware;
    private Handler mHandler;
    private Handler mHandlerUI;
    private HandlerThread mHandlerThread = new HandlerThread("Bluetooth");

    private BaseProduct mProduct;
    private DJIKey firmwareKey;
    private KeyListener firmwareVersionUpdater;
    private boolean hasStartedFirmVersionListener = false;
    private AtomicBoolean hasAppActivationListenerStarted = new AtomicBoolean(false);
    private static final int MSG_UPDATE_BLUETOOTH_CONNECTOR = 0;
    private static final int MSG_INFORM_ACTIVATION = 1;
    private static final int ACTIVATION_DALAY_TIME = 3000;
    private AppActivationState.AppActivationStateListener appActivationStateListener;
    private boolean isregisterForLDM = false;
    private Context mContext;

    public MainContent(Context context, AttributeSet attrs) {
        super(context, attrs);
        mContext = context;

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            permissionArrays = new String[]{
                    Manifest.permission.VIBRATE, // Gimbal rotation
                    Manifest.permission.INTERNET, // API requests
                    Manifest.permission.ACCESS_WIFI_STATE, // WIFI connected products
                    Manifest.permission.ACCESS_COARSE_LOCATION, // Maps
                    Manifest.permission.ACCESS_NETWORK_STATE, // WIFI connected products
                    Manifest.permission.ACCESS_FINE_LOCATION, // Maps
                    Manifest.permission.CHANGE_WIFI_STATE, // Changing between WIFI and USB connection
                    Manifest.permission.BLUETOOTH, // Bluetooth connected products
                    Manifest.permission.BLUETOOTH_ADMIN, // Bluetooth connected products
                    Manifest.permission.READ_PHONE_STATE, // Device UUID accessed upon registration
                    Manifest.permission.RECORD_AUDIO,// Speaker accessory
            };
        } else {//兼容Android 12
            permissionArrays = new String[]{
                    Manifest.permission.VIBRATE, // Gimbal rotation
                    Manifest.permission.INTERNET, // API requests
                    Manifest.permission.ACCESS_WIFI_STATE, // WIFI connected products
                    Manifest.permission.ACCESS_COARSE_LOCATION, // Maps
                    Manifest.permission.ACCESS_NETWORK_STATE, // WIFI connected products
                    Manifest.permission.ACCESS_FINE_LOCATION, // Maps
                    Manifest.permission.CHANGE_WIFI_STATE, // Changing between WIFI and USB connection
                    Manifest.permission.WRITE_EXTERNAL_STORAGE, // Log files
                    Manifest.permission.BLUETOOTH, // Bluetooth connected products
                    Manifest.permission.BLUETOOTH_ADMIN, // Bluetooth connected products
                    Manifest.permission.READ_EXTERNAL_STORAGE, // Log files
                    Manifest.permission.READ_PHONE_STATE, // Device UUID accessed upon registration
                    Manifest.permission.RECORD_AUDIO // Speaker accessory
            };
        }
    }

    @Override
    protected void onFinishInflate() {
        super.onFinishInflate();
        if (isInEditMode()) {
            return;
        }
        DJISampleApplication.getEventBus().register(this);
        initUI();
    }

    private void initUI() {
        Log.v(TAG, "initUI");

        progressBar = (ProgressBar) findViewById(R.id.progress_bar);
        mTextConnectionStatus = (TextView) findViewById(R.id.text_connection_status);
        mTextModelAvailable = (TextView) findViewById(R.id.text_model_available);
        mTextProduct = (TextView) findViewById(R.id.text_product_info);
        mBtnRegisterApp = (Button) findViewById(R.id.btn_registerApp);
        getmBtnRegisterAppForLDM = (Button) findViewById(R.id.btn_registerAppForLDM);
        mBtnOpen = (Button) findViewById(R.id.btn_open);
        mBridgeModeEditText = (EditText) findViewById(R.id.edittext_bridge_ip);
        mRosBridgeIpEditText = (EditText) findViewById(R.id.edittext_ros_bridge_ip);
        manualRosPcIp = getSavedRosPcIp();
        mRosBridgeIpEditText.setText(manualRosPcIp);
        mBridgePreviewFeed = null;
        mBtnBluetooth = (Button) findViewById(R.id.btn_bluetooth);
        mCheckboxFirmware = (CheckBox) findViewById(R.id.checkbox_firmware);

        //mBtnBluetooth.setEnabled(false);


        mBtnRegisterApp.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                isregisterForLDM = false;
                checkAndRequestPermissions();
            }
        });
        getmBtnRegisterAppForLDM.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                isregisterForLDM = true;
                checkAndRequestPermissions();
            }
        });
        mBtnOpen.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                if (GeneralUtils.isFastDoubleClick()) {
                    return;
                }
                DJISampleApplication.getEventBus().post(componentList);
            }
        });
        mBtnBluetooth.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                if (GeneralUtils.isFastDoubleClick()) {
                    return;
                }
                if (DJISampleApplication.getBluetoothProductConnector() == null) {
                    ToastUtils.setResultToToast("pls wait the sdk initiation finished");
                    return;
                }
                bluetoothView =
                        new ViewWrapper(new BluetoothView(getContext()), R.string.component_listview_bluetooth);
                DJISampleApplication.getEventBus().post(bluetoothView);
            }
        });
        mBridgeModeEditText.setOnEditorActionListener(new TextView.OnEditorActionListener() {
            @Override
            public boolean onEditorAction(TextView v, int actionId, KeyEvent event) {
                if (actionId == EditorInfo.IME_ACTION_SEARCH
                        || actionId == EditorInfo.IME_ACTION_DONE
                        || event != null
                        && event.getAction() == KeyEvent.ACTION_DOWN
                        && event.getKeyCode() == KeyEvent.KEYCODE_ENTER) {
                    if (event != null && event.isShiftPressed()) {
                        return false;
                    } else {
                        // the user is done typing.
                        handleBridgeIPTextChange();
                    }
                }
                return false; // pass on to other listeners.
            }
        });
        mBridgeModeEditText.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
            }

            @Override
            public void afterTextChanged(Editable s) {
                if (s != null && s.toString().contains("\n")) {
                    // the user is done typing.
                    // remove new line characcter
                    final String currentText = mBridgeModeEditText.getText().toString();
                    mBridgeModeEditText.setText(currentText.substring(0, currentText.indexOf('\n')));
                    handleBridgeIPTextChange();
                }
            }
        });
        mRosBridgeIpEditText.setOnEditorActionListener(new TextView.OnEditorActionListener() {
            @Override
            public boolean onEditorAction(TextView v, int actionId, KeyEvent event) {
                if (actionId == EditorInfo.IME_ACTION_SEARCH
                        || actionId == EditorInfo.IME_ACTION_DONE
                        || event != null
                        && event.getAction() == KeyEvent.ACTION_DOWN
                        && event.getKeyCode() == KeyEvent.KEYCODE_ENTER) {
                    if (event != null && event.isShiftPressed()) {
                        return false;
                    } else {
                        handleRosPcIpTextChange();
                    }
                }
                return false;
            }
        });
        mRosBridgeIpEditText.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
            }

            @Override
            public void afterTextChanged(Editable s) {
                if (s != null && s.toString().contains("\n")) {
                    final String currentText = mRosBridgeIpEditText.getText().toString();
                    mRosBridgeIpEditText.setText(currentText.substring(0, currentText.indexOf('\n')));
                    handleRosPcIpTextChange();
                } else if (s != null) {
                    manualRosPcIp = s.toString().trim();
                    saveRosPcIp(manualRosPcIp);
                }
            }
        });
        ((TextView) findViewById(R.id.text_version)).setText(getResources().getString(R.string.sdk_version,
                DJISDKManager.getInstance().getRegistrationSDKVersion()
                        + " Debug:"
                        + GlobalConfig.DEBUG));
    }

    private void handleBridgeIPTextChange() {
        // the user is done typing.
        final String bridgeIP = mBridgeModeEditText.getText().toString();
        DJISDKManager.getInstance().enableBridgeModeWithBridgeAppIP(bridgeIP);
        if (!TextUtils.isEmpty(bridgeIP)) {
            ToastUtils.setResultToToast("BridgeMode ON!\nIP: " + bridgeIP);
        }
    }

    private void handleRosPcIpTextChange() {
        final String rosPcIp = mRosBridgeIpEditText.getText().toString().trim();
        manualRosPcIp = rosPcIp;
        saveRosPcIp(rosPcIp);
        if (!TextUtils.isEmpty(rosPcIp)) {
            ToastUtils.setResultToToast("ROS bridge IP set:\n" + rosPcIp);
            retryDjiRosBridgeIfProductConnected();
        }
    }

    private String getSavedRosPcIp() {
        return getContext()
                .getSharedPreferences(getContext().getPackageName(), Context.MODE_PRIVATE)
                .getString(ROS_PC_IP_PREF_KEY, "");
    }

    private void saveRosPcIp(String rosPcIp) {
        getContext()
                .getSharedPreferences(getContext().getPackageName(), Context.MODE_PRIVATE)
                .edit()
                .putString(ROS_PC_IP_PREF_KEY, rosPcIp)
                .apply();
    }

    @Override
    protected void onAttachedToWindow() {
        Log.d(TAG, "Comes into the onAttachedToWindow");
        if (!isInEditMode()) {
            refreshSDKRelativeUI();
            mHandlerThread.start();
            final long currentTime = System.currentTimeMillis();
            mHandler = new Handler(mHandlerThread.getLooper()) {
                @Override
                public void handleMessage(Message msg) {
                    switch (msg.what) {
                        case MSG_UPDATE_BLUETOOTH_CONNECTOR:
                            //connected = DJISampleApplication.getBluetoothConnectStatus();
                            connector = DJISampleApplication.getBluetoothProductConnector();

                            if (connector != null) {
                                mBtnBluetooth.post(new Runnable() {
                                    @Override
                                    public void run() {
                                        mBtnBluetooth.setEnabled(true);
                                    }
                                });
                                return;
                            } else if ((System.currentTimeMillis() - currentTime) >= 5000) {
                                DialogUtils.showDialog(getContext(),
                                        "Fetch Connector failed, reboot if you want to connect the Bluetooth");
                                return;
                            } else if (connector == null) {
                                sendDelayMsg(0, MSG_UPDATE_BLUETOOTH_CONNECTOR);
                            }
                            break;
                        case MSG_INFORM_ACTIVATION:
                            loginToActivationIfNeeded();
                            break;
                    }
                }
            };
            mHandlerUI = new Handler(Looper.getMainLooper());
        }
        super.onAttachedToWindow();
    }

    private void sendDelayMsg(int msg, long delayMillis) {
        if (mHandler == null) {
            return;
        }

        if (!mHandler.hasMessages(msg)) {
            mHandler.sendEmptyMessageDelayed(msg, delayMillis);
        }
    }

    @Override
    protected void onDetachedFromWindow() {
        if (!isInEditMode()) {
            removeFirmwareVersionListener();
            removeAppActivationListenerIfNeeded();
            stopDjiRosBridge();
            mHandler.removeCallbacksAndMessages(null);
            mHandlerUI.removeCallbacksAndMessages(null);
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.JELLY_BEAN_MR2) {
                mHandlerThread.quitSafely();
            } else {
                mHandlerThread.quit();
            }
            mHandlerUI = null;
            mHandler = null;
        }
        super.onDetachedFromWindow();
    }

    private void updateVersion() {
        String version = null;
        if (mProduct != null) {
            version = mProduct.getFirmwarePackageVersion();
        }

        if (TextUtils.isEmpty(version)) {
            mTextModelAvailable.setText("Firmware version:N/A"); //Firmware version:
        } else {
            mTextModelAvailable.setText("Firmware version:" + version); //"Firmware version: " +
            removeFirmwareVersionListener();
        }
    }

    @Subscribe
    public void onConnectivityChange(MainActivity.ConnectivityChangeEvent event) {
        if (mHandlerUI != null) {
            mHandlerUI.post(new Runnable() {
                @Override
                public void run() {
                    refreshSDKRelativeUI();
                }
            });
        }
    }

    private void refreshSDKRelativeUI() {
        mProduct = DJISampleApplication.getProductInstance();
        Log.d(TAG, "mProduct: " + (mProduct == null ? "null" : "unnull"));
        if (null != mProduct) {
            if (mProduct.isConnected()) {
                mBtnOpen.setEnabled(true);
                String str = mProduct instanceof Aircraft ? "DJIAircraft" : "DJIHandHeld";
                mTextConnectionStatus.setText("Status: " + str + " connected");
                tryUpdateFirmwareVersionWithListener();
                if (mProduct instanceof Aircraft) {
                    addAppActivationListenerIfNeeded();
                }

                if (null != mProduct.getModel()) {
                    mTextProduct.setText("" + mProduct.getModel().getDisplayName());
                } else {
                    mTextProduct.setText(R.string.product_information);
                }
            } else if (mProduct instanceof Aircraft) {
                Aircraft aircraft = (Aircraft) mProduct;
                if (aircraft.getRemoteController() != null && aircraft.getRemoteController().isConnected()) {
                    mTextConnectionStatus.setText(R.string.connection_only_rc);
                    mTextProduct.setText(R.string.product_information);
                    mBtnOpen.setEnabled(false);
                    mTextModelAvailable.setText("Firmware version:N/A");
                }
            }
        } else {
            mBtnOpen.setEnabled(false);
            mTextProduct.setText(R.string.product_information);
            mTextConnectionStatus.setText(R.string.connection_loose);
            mTextModelAvailable.setText("Firmware version:N/A");
        }
    }

    private void tryUpdateFirmwareVersionWithListener() {
        if (!hasStartedFirmVersionListener) {
            firmwareVersionUpdater = new KeyListener() {
                @Override
                public void onValueChange(final Object o, final Object o1) {
                    mHandlerUI.post(new Runnable() {
                        @Override
                        public void run() {
                            updateVersion();
                        }
                    });
                }
            };
            firmwareKey = ProductKey.create(ProductKey.FIRMWARE_PACKAGE_VERSION);
            if (KeyManager.getInstance() != null) {
                KeyManager.getInstance().addListener(firmwareKey, firmwareVersionUpdater);
            }
            hasStartedFirmVersionListener = true;
        }
        updateVersion();
    }

    private void removeFirmwareVersionListener() {
        if (hasStartedFirmVersionListener) {
            if (KeyManager.getInstance() != null) {
                KeyManager.getInstance().removeListener(firmwareVersionUpdater);
            }
        }
        hasStartedFirmVersionListener = false;
    }

    private void addAppActivationListenerIfNeeded() {
        if (AppActivationManager.getInstance().getAppActivationState() != AppActivationState.ACTIVATED) {
            sendDelayMsg(MSG_INFORM_ACTIVATION, ACTIVATION_DALAY_TIME);
            if (hasAppActivationListenerStarted.compareAndSet(false, true)) {
                appActivationStateListener = new AppActivationState.AppActivationStateListener() {

                    @Override
                    public void onUpdate(AppActivationState appActivationState) {
                        if (mHandler != null && mHandler.hasMessages(MSG_INFORM_ACTIVATION)) {
                            mHandler.removeMessages(MSG_INFORM_ACTIVATION);
                        }
                        if (appActivationState != AppActivationState.ACTIVATED) {
                            sendDelayMsg(MSG_INFORM_ACTIVATION, ACTIVATION_DALAY_TIME);
                        }
                    }
                };
                AppActivationManager.getInstance().addAppActivationStateListener(appActivationStateListener);
            }
        }
    }

    private void removeAppActivationListenerIfNeeded() {
        if (hasAppActivationListenerStarted.compareAndSet(true, false)) {
            AppActivationManager.getInstance().removeAppActivationStateListener(appActivationStateListener);
        }
    }

    private void loginToActivationIfNeeded() {
        if (AppActivationManager.getInstance().getAppActivationState() == AppActivationState.LOGIN_REQUIRED) {
            UserAccountManager.getInstance()
                    .logIntoDJIUserAccount(getContext(),
                            new CommonCallbacks.CompletionCallbackWith<UserAccountState>() {
                                @Override
                                public void onSuccess(UserAccountState userAccountState) {
                                    ToastUtils.setResultToToast("Login Successed!");
                                }

                                @Override
                                public void onFailure(DJIError djiError) {
                                    ToastUtils.setResultToToast("Login Failed!");
                                }
                            });
        }
    }

    //region Registration n' Permissions Helpers

    /**
     * Checks if there is any missing permissions, and
     * requests runtime permission if needed.
     */
    private void checkAndRequestPermissions() {
        // Check for permissions
        List<String> missingPermission = new ArrayList<>();
        for (String eachPermission : permissionArrays) {
            if (ContextCompat.checkSelfPermission(mContext, eachPermission) != PackageManager.PERMISSION_GRANTED) {
                missingPermission.add(eachPermission);
            }
        }
        // Request for missing permissions
        if (missingPermission.isEmpty()) {
            startSDKRegistration();
        } else if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            ActivityCompat.requestPermissions((Activity) mContext,
                    missingPermission.toArray(new String[missingPermission.size()]),
                    REQUEST_PERMISSION_CODE);
        }
    }

    private void startSDKRegistration() {
        if (isRegistrationInProgress.compareAndSet(false, true)) {
            AsyncTask.execute(new Runnable() {
                @Override
                public void run() {
                    ToastUtils.setResultToToast(mContext.getString(R.string.sdk_registration_doing_message));
                    //if we hope the Firmware Upgrade module could access the network under LDM mode, we need call the setModuleNetworkServiceEnabled()
                    //method before the registerAppForLDM() method
                    if (mCheckboxFirmware.isChecked()) {
                        DJISDKManager.getInstance().getLDMManager().setModuleNetworkServiceEnabled(new LDMModule.Builder().moduleType(
                                LDMModuleType.FIRMWARE_UPGRADE).enabled(true).build());
                    } else {
                        DJISDKManager.getInstance().getLDMManager().setModuleNetworkServiceEnabled(new LDMModule.Builder().moduleType(
                                LDMModuleType.FIRMWARE_UPGRADE).enabled(false).build());
                    }
                    if (isregisterForLDM) {
                        DJISDKManager.getInstance().registerAppForLDM(mContext.getApplicationContext(), new DJISDKManager.SDKManagerCallback() {
                            @Override
                            public void onRegister(DJIError djiError) {
                                if (djiError == DJISDKError.REGISTRATION_SUCCESS) {
                                    DJILog.e("App registration for LDM", DJISDKError.REGISTRATION_SUCCESS.getDescription());
                                    DJISDKManager.getInstance().startConnectionToProduct();
                                    ToastUtils.setResultToToast(mContext.getString(R.string.sdk_registration_success_message));
                                } else {
                                    ToastUtils.setResultToToast(mContext.getString(R.string.sdk_registration_message) + djiError.getDescription());
                                }
                                Log.v(TAG, djiError.getDescription());
                                hideProcess();
                            }

                            @Override
                            public void onProductDisconnect() {
                                Log.d(TAG, "onProductDisconnect");
                                stopDjiRosBridge();
                                notifyStatusChange();
                            }

                            @Override
                            public void onProductConnect(BaseProduct baseProduct) {
                                Log.d(TAG, String.format("onProductConnect newProduct:%s", baseProduct));
                                post(new Runnable() {
                                    @Override
                                    public void run() {
                                        startDjiRosBridge();
                                    }
                                });
                                notifyStatusChange();
                            }

                            @Override
                            public void onProductChanged(BaseProduct baseProduct) {
                                notifyStatusChange();
                            }

                            @Override
                            public void onComponentChange(BaseProduct.ComponentKey componentKey,
                                                          BaseComponent oldComponent,
                                                          BaseComponent newComponent) {
                                if (newComponent != null) {
                                    newComponent.setComponentListener(mDJIComponentListener);

                                    if (componentKey == BaseProduct.ComponentKey.FLIGHT_CONTROLLER) {
                                        showDBVersion();
                                    }
                                }
                                Log.d(TAG,
                                        String.format("onComponentChange key:%s, oldComponent:%s, newComponent:%s",
                                                componentKey,
                                                oldComponent,
                                                newComponent));

                                notifyStatusChange();
                            }

                            @Override
                            public void onInitProcess(DJISDKInitEvent djisdkInitEvent, int i) {

                            }

                            @Override
                            public void onDatabaseDownloadProgress(long current, long total) {
                                int process = (int) (100 * current / total);
                                if (process == lastProcess) {
                                    return;
                                }
                                lastProcess = process;
                                showProgress(process);
                                if (process % 25 == 0) {
                                    ToastUtils.setResultToToast("DB load process : " + process);
                                } else if (process == 0) {
                                    ToastUtils.setResultToToast("DB load begin");
                                }
                            }
                        });

                    } else {
                        DJISDKManager.getInstance().registerApp(mContext.getApplicationContext(), new DJISDKManager.SDKManagerCallback() {
                            @Override
                            public void onRegister(DJIError djiError) {
                                if (djiError == DJISDKError.REGISTRATION_SUCCESS) {
                                    DJILog.e("App registration", DJISDKError.REGISTRATION_SUCCESS.getDescription());
                                    DJISDKManager.getInstance().startConnectionToProduct();
                                    ToastUtils.setResultToToast(mContext.getString(R.string.sdk_registration_success_message));
                                } else {
                                    ToastUtils.setResultToToast(mContext.getString(R.string.sdk_registration_message) + djiError.getDescription());
                                }
                                Log.v(TAG, djiError.getDescription());
                                hideProcess();
                            }

                            @Override
                            public void onProductDisconnect() {
                                Log.d(TAG, "onProductDisconnect");
                                stopDjiRosBridge();
                                notifyStatusChange();
                            }

                            @Override
                            public void onProductConnect(BaseProduct baseProduct) {
                                Log.d(TAG, String.format("onProductConnect newProduct:%s", baseProduct));
                                post(new Runnable() {
                                    @Override
                                    public void run() {
                                        startDjiRosBridge();
                                    }
                                });
                                notifyStatusChange();
                            }

                            @Override
                            public void onProductChanged(BaseProduct baseProduct) {
                                notifyStatusChange();
                            }

                            @Override
                            public void onComponentChange(BaseProduct.ComponentKey componentKey,
                                                          BaseComponent oldComponent,
                                                          BaseComponent newComponent) {
                                if (newComponent != null) {
                                    newComponent.setComponentListener(mDJIComponentListener);

                                    if (componentKey == BaseProduct.ComponentKey.FLIGHT_CONTROLLER) {
                                        showDBVersion();
                                    }
                                }
                                Log.d(TAG,
                                        String.format("onComponentChange key:%s, oldComponent:%s, newComponent:%s",
                                                componentKey,
                                                oldComponent,
                                                newComponent));

                                notifyStatusChange();
                            }

                            @Override
                            public void onInitProcess(DJISDKInitEvent djisdkInitEvent, int i) {

                            }

                            @Override
                            public void onDatabaseDownloadProgress(long current, long total) {
                                int process = (int) (100 * current / total);
                                if (process == lastProcess) {
                                    return;
                                }
                                lastProcess = process;
                                showProgress(process);
                                if (process % 25 == 0) {
                                    ToastUtils.setResultToToast("DB load process : " + process);
                                } else if (process == 0) {
                                    ToastUtils.setResultToToast("DB load begin");
                                }
                            }
                        });

                    }
                }
            });
        }
    }

    private void showProgress(final int process) {
        mHander.post(new Runnable() {
            @Override
            public void run() {
                progressBar.setVisibility(View.VISIBLE);
                progressBar.setProgress(process);
            }
        });
    }

    private void showDBVersion() {
        mHander.postDelayed(new Runnable() {
            @Override
            public void run() {
                DJISDKManager.getInstance().getFlyZoneManager().getPreciseDatabaseVersion(new CommonCallbacks.CompletionCallbackWith<String>() {
                    @Override
                    public void onSuccess(String s) {
                        ToastUtils.setResultToToast("db load success ! version : " + s);
                    }

                    @Override
                    public void onFailure(DJIError djiError) {
                        ToastUtils.setResultToToast("db load failure ! get version error : " + djiError.getDescription());

                    }
                });
            }
        }, 3000);
    }

    private void hideProcess() {
        mHander.post(new Runnable() {
            @Override
            public void run() {
                progressBar.setVisibility(View.GONE);
            }
        });
    }

    private void notifyStatusChange() {
        DJISampleApplication.getEventBus().post(new MainActivity.ConnectivityChangeEvent());
    }
    //endregion

    private synchronized void startDjiRosBridge() {
        if (bridgeStartupRunning || gimbalBridgeStarted || positionBridgeStarted || videoBridgeStarted) {
            return;
        }

        bridgeStartupRunning = true;
        final int startupGeneration = ++bridgeStartGeneration;

        bridgeExecutor.execute(new Runnable() {
            @Override
            public void run() {
                final InetAddress bridgeAddress = discoverRosBridgeAddress();
                if (bridgeAddress == null) {
                    synchronized (MainContent.this) {
                        if (startupGeneration == bridgeStartGeneration) {
                            bridgeStartupRunning = false;
                        }
                    }
                    showBridgeToast("Discovery failed. Enter ROS PC IP.");
                    retryDjiRosBridgeIfProductConnected();
                    return;
                }

                synchronized (MainContent.this) {
                    if (!bridgeStartupRunning || startupGeneration != bridgeStartGeneration) {
                        return;
                    }
                    rosPcAddress = bridgeAddress;
                }

                post(new Runnable() {
                    @Override
                    public void run() {
                        synchronized (MainContent.this) {
                            if (!bridgeStartupRunning || startupGeneration != bridgeStartGeneration) {
                                return;
                            }
                        }

                        startGimbalUdpBridge();
                        startPositionUdpBridge();
                        startVideoTcpBridge();

                        synchronized (MainContent.this) {
                            if (startupGeneration == bridgeStartGeneration) {
                                bridgeStartupRunning = false;
                            }
                        }
                    }
                });
            }
        });
    }

    private synchronized void stopDjiRosBridge() {
        bridgeStartGeneration++;
        bridgeStartupRunning = false;
        stopGimbalUdpBridge();
        stopPositionUdpBridge();
        stopVideoTcpBridge();
        rosPcAddress = null;
    }

    private InetAddress discoverRosBridgeAddress() {
        InetAddress manualBridgeAddress = getManualBridgeAddress();
        if (manualBridgeAddress != null) {
            return manualBridgeAddress;
        }

        DatagramSocket socket = null;
        try {
            socket = new DatagramSocket();
            socket.setBroadcast(true);
            socket.setSoTimeout(DISCOVERY_TIMEOUT_MS);

            byte[] request = DISCOVERY_REQUEST.getBytes(StandardCharsets.UTF_8);
            byte[] responseBuffer = new byte[512];
            for (int attempt = 0; attempt < DISCOVERY_ATTEMPTS; attempt++) {
                sendDiscoveryRequests(socket, request);

                try {
                    DatagramPacket responsePacket =
                            new DatagramPacket(responseBuffer, responseBuffer.length);
                    socket.receive(responsePacket);
                    String response = new String(
                            responsePacket.getData(),
                            0,
                            responsePacket.getLength(),
                            StandardCharsets.UTF_8
                    ).trim();

                    if (response.startsWith(DISCOVERY_RESPONSE_PREFIX)) {
                        Log.i(TAG, "DJI ROS bridge discovered at "
                                + responsePacket.getAddress().getHostAddress());
                        return responsePacket.getAddress();
                    }
                } catch (SocketTimeoutException ignored) {
                }
            }
        } catch (Exception e) {
            Log.w(TAG, "DJI ROS bridge discovery error", e);
        } finally {
            if (socket != null) {
                socket.close();
            }
        }

        return null;
    }

    private InetAddress getManualBridgeAddress() {
        String bridgeIp = manualRosPcIp;
        if (TextUtils.isEmpty(bridgeIp)) {
            return null;
        }

        try {
            InetAddress address = InetAddress.getByName(bridgeIp);
            Log.i(TAG, "Using manual DJI ROS bridge IP " + address.getHostAddress());
            showBridgeToast("Using ROS PC IP: " + address.getHostAddress());
            return address;
        } catch (Exception e) {
            Log.w(TAG, "Invalid manual DJI ROS bridge IP: " + bridgeIp, e);
            showBridgeToast("Invalid ROS bridge IP: " + bridgeIp);
            return null;
        }
    }

    private void retryDjiRosBridgeIfProductConnected() {
        postDelayed(new Runnable() {
            @Override
            public void run() {
                BaseProduct product = DJISDKManager.getInstance().getProduct();
                if (product != null && product.isConnected()) {
                    startDjiRosBridge();
                }
            }
        }, DISCOVERY_RETRY_DELAY_MS);
    }

    private void sendDiscoveryRequests(DatagramSocket socket, byte[] request) throws Exception {
        List<InetAddress> broadcastAddresses = new ArrayList<>();
        broadcastAddresses.add(InetAddress.getByName("255.255.255.255"));

        Enumeration<NetworkInterface> interfaces = NetworkInterface.getNetworkInterfaces();
        while (interfaces != null && interfaces.hasMoreElements()) {
            NetworkInterface networkInterface = interfaces.nextElement();
            try {
                if (!networkInterface.isUp() || networkInterface.isLoopback()) {
                    continue;
                }

                for (InterfaceAddress interfaceAddress : networkInterface.getInterfaceAddresses()) {
                    InetAddress broadcast = interfaceAddress.getBroadcast();
                    if (broadcast != null) {
                        broadcastAddresses.add(broadcast);
                    }
                }
            } catch (Exception e) {
                Log.d(TAG, "Skipping interface during discovery: " + networkInterface.getName(), e);
            }
        }

        for (InetAddress broadcastAddress : broadcastAddresses) {
            try {
                DatagramPacket requestPacket = new DatagramPacket(
                        request,
                        request.length,
                        broadcastAddress,
                        DISCOVERY_UDP_PORT
                );
                socket.send(requestPacket);
            } catch (Exception e) {
                Log.d(TAG, "Discovery send failed for " + broadcastAddress, e);
            }
        }
    }

    private void showBridgeToast(final String message) {
        post(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(getContext(), message, Toast.LENGTH_LONG).show();
            }
        });
    }

    private synchronized void startGimbalUdpBridge() {
        if (gimbalBridgeStarted) {
            return;
        }

        BaseProduct product = DJISDKManager.getInstance().getProduct();

        if (!(product instanceof Aircraft)) {
            Toast.makeText(getContext(), "No aircraft connected yet", Toast.LENGTH_LONG).show();
            return;
        }

        Aircraft aircraft = (Aircraft) product;

        if (aircraft.getGimbal() == null) {
            Toast.makeText(getContext(), "Gimbal is null", Toast.LENGTH_LONG).show();
            return;
        }

        if (KeyManager.getInstance() == null) {
            Toast.makeText(getContext(), "KeyManager is null", Toast.LENGTH_LONG).show();
            return;
        }

        InetAddress bridgeAddress = rosPcAddress;
        if (bridgeAddress == null) {
            Toast.makeText(getContext(), "DJI ROS bridge address is unknown", Toast.LENGTH_LONG).show();
            return;
        }

        DatagramSocket socket = null;
        try {
            socket = new DatagramSocket();
            gimbalSocket = socket;
        } catch (Exception e) {
            if (socket != null) {
                socket.close();
            }
            Toast.makeText(getContext(), "UDP init failed: " + e.getMessage(), Toast.LENGTH_LONG).show();
            return;
        }

        gimbalAttitudeKey = GimbalKey.create(GimbalKey.ATTITUDE_IN_DEGREES);
        Object initialAttitude = KeyManager.getInstance().getValue(gimbalAttitudeKey);
        if (initialAttitude instanceof Attitude) {
            latestGimbalAttitude = (Attitude) initialAttitude;
        }
        gimbalAttitudeUpdater = new KeyListener() {
            @Override
            public void onValueChange(Object oldValue, Object newValue) {
                if (newValue instanceof Attitude) {
                    latestGimbalAttitude = (Attitude) newValue;
                }
            }
        };
        KeyManager.getInstance().addListener(gimbalAttitudeKey, gimbalAttitudeUpdater);
        gimbalSendTask = udpExecutor.scheduleAtFixedRate(new Runnable() {
            @Override
            public void run() {
                pollAndSendGimbalAttitude();
            }
        }, 0, 100, TimeUnit.MILLISECONDS);
        gimbalBridgeStarted = true;

        Toast.makeText(
                getContext(),
                "Gimbal UDP bridge started: " + bridgeAddress.getHostAddress(),
                Toast.LENGTH_LONG
        ).show();
    }

    private synchronized void stopGimbalUdpBridge() {
        if (gimbalSendTask != null) {
            gimbalSendTask.cancel(true);
            gimbalSendTask = null;
        }
        if (gimbalAttitudeUpdater != null && KeyManager.getInstance() != null) {
            KeyManager.getInstance().removeListener(gimbalAttitudeUpdater);
        }
        gimbalAttitudeUpdater = null;
        gimbalAttitudeKey = null;
        latestGimbalAttitude = null;
        if (gimbalSocket != null) {
            gimbalSocket.close();
            gimbalSocket = null;
        }
        gimbalBridgeStarted = false;
    }

    private void pollAndSendGimbalAttitude() {
        DJIKey key = gimbalAttitudeKey;
        KeyManager keyManager = KeyManager.getInstance();
        if (key != null && keyManager != null) {
            Object currentAttitude = keyManager.getValue(key);
            if (currentAttitude instanceof Attitude) {
                latestGimbalAttitude = (Attitude) currentAttitude;
            }
        }

        Attitude attitude = latestGimbalAttitude;
        if (attitude == null) {
            return;
        }

        long now = System.currentTimeMillis();
        String json = String.format(
                Locale.US,
                "{\"t\":%d,\"pitch\":%.3f,\"roll\":%.3f,\"yaw\":%.3f}",
                now,
                attitude.getPitch(),
                attitude.getRoll(),
                attitude.getYaw()
        );

        sendUdp(json);
    }

    private void sendUdp(String message) {
        try {
            DatagramSocket socket = gimbalSocket;
            InetAddress address = rosPcAddress;
            if (socket == null || address == null || socket.isClosed()) {
                return;
            }
            byte[] data = message.getBytes(StandardCharsets.UTF_8);
            DatagramPacket packet = new DatagramPacket(
                    data,
                    data.length,
                    address,
                    GIMBAL_UDP_PORT
            );
            socket.send(packet);
        } catch (Exception ignored) {
        }
    }

    private synchronized void startPositionUdpBridge() {
        if (positionBridgeStarted) {
            return;
        }

        BaseProduct product = DJISDKManager.getInstance().getProduct();
        if (!(product instanceof Aircraft)) {
            Toast.makeText(getContext(), "No aircraft connected yet", Toast.LENGTH_LONG).show();
            return;
        }

        KeyManager keyManager = KeyManager.getInstance();
        if (keyManager == null) {
            Toast.makeText(getContext(), "KeyManager is null", Toast.LENGTH_LONG).show();
            return;
        }

        InetAddress bridgeAddress = rosPcAddress;
        if (bridgeAddress == null) {
            Toast.makeText(getContext(), "DJI ROS bridge address is unknown", Toast.LENGTH_LONG).show();
            return;
        }

        try {
            positionSocket = new DatagramSocket();
            aircraftLocationKey = FlightControllerKey.create(FlightControllerKey.AIRCRAFT_LOCATION);
            velocityXKey = FlightControllerKey.create(FlightControllerKey.VELOCITY_X);
            velocityYKey = FlightControllerKey.create(FlightControllerKey.VELOCITY_Y);
            velocityZKey = FlightControllerKey.create(FlightControllerKey.VELOCITY_Z);
            satelliteCountKey = FlightControllerKey.create(FlightControllerKey.SATELLITE_COUNT);
            gpsSignalLevelKey = FlightControllerKey.create(FlightControllerKey.GPS_SIGNAL_LEVEL);
            rtkStateKey = FlightControllerKey.create(FlightControllerKey.RTK_STATE);

            updateAircraftLocation(getKeyValue(keyManager, aircraftLocationKey));
            updateVelocityX(getKeyValue(keyManager, velocityXKey));
            updateVelocityY(getKeyValue(keyManager, velocityYKey));
            updateVelocityZ(getKeyValue(keyManager, velocityZKey));
            updateSatelliteCount(getKeyValue(keyManager, satelliteCountKey));
            updateGpsSignalLevel(getKeyValue(keyManager, gpsSignalLevelKey));
            updateRtkState(getKeyValue(keyManager, rtkStateKey));

            aircraftLocationUpdater = new KeyListener() {
                @Override
                public void onValueChange(Object oldValue, Object newValue) {
                    updateAircraftLocation(newValue);
                }
            };
            velocityXUpdater = new KeyListener() {
                @Override
                public void onValueChange(Object oldValue, Object newValue) {
                    updateVelocityX(newValue);
                }
            };
            velocityYUpdater = new KeyListener() {
                @Override
                public void onValueChange(Object oldValue, Object newValue) {
                    updateVelocityY(newValue);
                }
            };
            velocityZUpdater = new KeyListener() {
                @Override
                public void onValueChange(Object oldValue, Object newValue) {
                    updateVelocityZ(newValue);
                }
            };
            satelliteCountUpdater = new KeyListener() {
                @Override
                public void onValueChange(Object oldValue, Object newValue) {
                    updateSatelliteCount(newValue);
                }
            };
            gpsSignalLevelUpdater = new KeyListener() {
                @Override
                public void onValueChange(Object oldValue, Object newValue) {
                    updateGpsSignalLevel(newValue);
                }
            };
            rtkStateUpdater = new KeyListener() {
                @Override
                public void onValueChange(Object oldValue, Object newValue) {
                    updateRtkState(newValue);
                }
            };

            addKeyListener(keyManager, aircraftLocationKey, aircraftLocationUpdater, "aircraft_location");
            addKeyListener(keyManager, velocityXKey, velocityXUpdater, "velocity_x");
            addKeyListener(keyManager, velocityYKey, velocityYUpdater, "velocity_y");
            addKeyListener(keyManager, velocityZKey, velocityZUpdater, "velocity_z");
            addKeyListener(keyManager, satelliteCountKey, satelliteCountUpdater, "satellite_count");
            addKeyListener(keyManager, gpsSignalLevelKey, gpsSignalLevelUpdater, "gps_signal_level");
            addKeyListener(keyManager, rtkStateKey, rtkStateUpdater, "rtk_state");

            positionSendTask = udpExecutor.scheduleAtFixedRate(new Runnable() {
                @Override
                public void run() {
                    pollAndSendPositionTelemetry();
                }
            }, 0, 100, TimeUnit.MILLISECONDS);

            positionBridgeStarted = true;
        } catch (Exception e) {
            Log.w(TAG, "Position UDP bridge init failed", e);
            stopPositionUdpBridge();
            Toast.makeText(getContext(), "Position UDP init failed: " + e.getMessage(), Toast.LENGTH_LONG).show();
            return;
        }

        Toast.makeText(
                getContext(),
                "Position UDP bridge started: " + bridgeAddress.getHostAddress(),
                Toast.LENGTH_LONG
        ).show();
    }

    private void addKeyListener(
            KeyManager keyManager,
            DJIKey key,
            KeyListener keyListener,
            String name
    ) {
        if (keyManager == null || key == null || keyListener == null) {
            return;
        }

        try {
            keyManager.addListener(key, keyListener);
        } catch (Exception e) {
            Log.d(TAG, "Position key listener add failed for " + name, e);
        }
    }

    private synchronized void stopPositionUdpBridge() {
        if (positionSendTask != null) {
            positionSendTask.cancel(true);
            positionSendTask = null;
        }

        KeyManager keyManager = KeyManager.getInstance();
        removeKeyListener(keyManager, aircraftLocationUpdater);
        removeKeyListener(keyManager, velocityXUpdater);
        removeKeyListener(keyManager, velocityYUpdater);
        removeKeyListener(keyManager, velocityZUpdater);
        removeKeyListener(keyManager, satelliteCountUpdater);
        removeKeyListener(keyManager, gpsSignalLevelUpdater);
        removeKeyListener(keyManager, rtkStateUpdater);

        aircraftLocationKey = null;
        velocityXKey = null;
        velocityYKey = null;
        velocityZKey = null;
        satelliteCountKey = null;
        gpsSignalLevelKey = null;
        rtkStateKey = null;
        aircraftLocationUpdater = null;
        velocityXUpdater = null;
        velocityYUpdater = null;
        velocityZUpdater = null;
        satelliteCountUpdater = null;
        gpsSignalLevelUpdater = null;
        rtkStateUpdater = null;
        latestAircraftLocation = null;
        latestVelocityX = null;
        latestVelocityY = null;
        latestVelocityZ = null;
        latestSatelliteCount = null;
        latestGpsSignalLevel = null;
        latestRtkState = null;

        if (positionSocket != null) {
            positionSocket.close();
            positionSocket = null;
        }
        positionBridgeStarted = false;
    }

    private void removeKeyListener(KeyManager keyManager, KeyListener keyListener) {
        if (keyManager == null || keyListener == null) {
            return;
        }

        try {
            keyManager.removeListener(keyListener);
        } catch (Exception e) {
            Log.d(TAG, "Position key listener removal failed", e);
        }
    }

    private void pollAndSendPositionTelemetry() {
        KeyManager keyManager = KeyManager.getInstance();
        if (keyManager != null) {
            updateAircraftLocation(getKeyValue(keyManager, aircraftLocationKey));
            updateVelocityX(getKeyValue(keyManager, velocityXKey));
            updateVelocityY(getKeyValue(keyManager, velocityYKey));
            updateVelocityZ(getKeyValue(keyManager, velocityZKey));
            updateSatelliteCount(getKeyValue(keyManager, satelliteCountKey));
            updateGpsSignalLevel(getKeyValue(keyManager, gpsSignalLevelKey));
            updateRtkState(getKeyValue(keyManager, rtkStateKey));
        }

        sendPositionUdp(buildPositionJson(System.currentTimeMillis(), latestAircraftLocation));
    }

    private Object getKeyValue(KeyManager keyManager, DJIKey key) {
        if (keyManager == null || key == null) {
            return null;
        }

        try {
            return keyManager.getValue(key);
        } catch (Exception e) {
            Log.d(TAG, "Position key read failed", e);
            return null;
        }
    }

    private void updateAircraftLocation(Object value) {
        if (value instanceof LocationCoordinate3D) {
            latestAircraftLocation = (LocationCoordinate3D) value;
        }
    }

    private void updateVelocityX(Object value) {
        latestVelocityX = asFloat(value);
    }

    private void updateVelocityY(Object value) {
        latestVelocityY = asFloat(value);
    }

    private void updateVelocityZ(Object value) {
        latestVelocityZ = asFloat(value);
    }

    private void updateSatelliteCount(Object value) {
        latestSatelliteCount = asInteger(value);
    }

    private void updateGpsSignalLevel(Object value) {
        if (value != null) {
            latestGpsSignalLevel = value;
        }
    }

    private void updateRtkState(Object value) {
        if (value instanceof RTKState) {
            latestRtkState = (RTKState) value;
        }
    }

    private Float asFloat(Object value) {
        if (value instanceof Number) {
            return ((Number) value).floatValue();
        }
        return null;
    }

    private Integer asInteger(Object value) {
        if (value instanceof Number) {
            return ((Number) value).intValue();
        }
        return null;
    }

    private String buildPositionJson(long now, LocationCoordinate3D aircraftLocation) {
        StringBuilder builder = new StringBuilder(512);
        boolean positionValid = false;
        String positionStatus = "waiting_for_aircraft_location";
        double latitude = Double.NaN;
        double longitude = Double.NaN;
        double altitude = Double.NaN;

        if (aircraftLocation != null) {
            try {
                latitude = aircraftLocation.getLatitude();
                longitude = aircraftLocation.getLongitude();
                altitude = aircraftLocation.getAltitude();
                positionValid = isFinite(latitude) && isFinite(longitude);
                positionStatus = positionValid ? "ok" : "invalid_aircraft_location";
            } catch (Exception e) {
                positionStatus = "aircraft_location_read_failed";
            }
        }

        builder.append("{\"t\":").append(now);
        appendJsonString(builder, "source", "flight_controller");
        appendJsonBoolean(builder, "position_valid", positionValid);
        appendJsonString(builder, "position_status", positionStatus);
        appendJsonDouble(builder, "lat", latitude, 8);
        appendJsonDouble(builder, "lon", longitude, 8);
        appendJsonDouble(builder, "alt", altitude, 3);
        appendJsonFloat(builder, "vx", latestVelocityX, 3);
        appendJsonFloat(builder, "vy", latestVelocityY, 3);
        appendJsonFloat(builder, "vz", latestVelocityZ, 3);
        appendJsonInt(builder, "satellites", latestSatelliteCount);
        appendJsonString(builder, "gps_signal", safeName(latestGpsSignalLevel));
        appendRtkJson(builder, latestRtkState);
        builder.append("}");
        return builder.toString();
    }

    private void appendRtkJson(StringBuilder builder, RTKState rtkState) {
        if (rtkState == null) {
            appendJsonBoolean(builder, "rtk_available", false);
            return;
        }

        appendJsonBoolean(builder, "rtk_available", true);
        try {
            appendJsonBoolean(builder, "rtk_used", rtkState.isRTKBeingUsed());
        } catch (Exception e) {
            appendJsonNull(builder, "rtk_used");
        }
        try {
            appendJsonString(builder, "rtk_solution", safeName(rtkState.getPositioningSolution()));
        } catch (Exception e) {
            appendJsonString(builder, "rtk_solution", null);
        }
        try {
            appendJsonBoolean(builder, "rtk_heading_valid", rtkState.isHeadingValid());
        } catch (Exception e) {
            appendJsonNull(builder, "rtk_heading_valid");
        }
        try {
            appendJsonDouble(builder, "rtk_heading", rtkState.getFusionHeading(), 3);
        } catch (Exception e) {
            appendJsonNull(builder, "rtk_heading");
        }

        LocationCoordinate2D fusionLocation = null;
        try {
            fusionLocation = rtkState.getFusionMobileStationLocation();
        } catch (Exception e) {
            Log.d(TAG, "RTK fusion location read failed", e);
        }

        if (fusionLocation != null) {
            appendJsonDouble(builder, "rtk_lat", fusionLocation.getLatitude(), 8);
            appendJsonDouble(builder, "rtk_lon", fusionLocation.getLongitude(), 8);
        } else {
            appendJsonNull(builder, "rtk_lat");
            appendJsonNull(builder, "rtk_lon");
        }

        try {
            appendJsonDouble(builder, "rtk_alt", rtkState.getFusionMobileStationAltitude(), 3);
        } catch (Exception e) {
            appendJsonNull(builder, "rtk_alt");
        }
        try {
            appendJsonInt(builder, "rtk_satellites", rtkState.getSatelliteCount());
        } catch (Exception e) {
            appendJsonNull(builder, "rtk_satellites");
        }
        try {
            appendJsonInt(builder, "rtk_mobile_station_1_satellites", rtkState.getMobileStation1SatelliteCount());
        } catch (Exception e) {
            appendJsonNull(builder, "rtk_mobile_station_1_satellites");
        }
        try {
            appendJsonInt(builder, "rtk_mobile_station_2_satellites", rtkState.getMobileStation2SatelliteCount());
        } catch (Exception e) {
            appendJsonNull(builder, "rtk_mobile_station_2_satellites");
        }
    }

    private void appendJsonString(StringBuilder builder, String name, String value) {
        builder.append(",\"").append(name).append("\":");
        if (TextUtils.isEmpty(value)) {
            builder.append("null");
            return;
        }
        builder.append("\"").append(escapeJson(value)).append("\"");
    }

    private void appendJsonInt(StringBuilder builder, String name, Integer value) {
        builder.append(",\"").append(name).append("\":");
        if (value == null) {
            builder.append("null");
        } else {
            builder.append(value);
        }
    }

    private void appendJsonInt(StringBuilder builder, String name, int value) {
        builder.append(",\"").append(name).append("\":").append(value);
    }

    private void appendJsonFloat(StringBuilder builder, String name, Float value, int decimals) {
        builder.append(",\"").append(name).append("\":");
        if (value == null || !isFinite(value)) {
            builder.append("null");
        } else {
            builder.append(String.format(Locale.US, "%." + decimals + "f", value));
        }
    }

    private void appendJsonDouble(StringBuilder builder, String name, double value, int decimals) {
        builder.append(",\"").append(name).append("\":");
        if (!isFinite(value)) {
            builder.append("null");
        } else {
            builder.append(String.format(Locale.US, "%." + decimals + "f", value));
        }
    }

    private void appendJsonBoolean(StringBuilder builder, String name, boolean value) {
        builder.append(",\"").append(name).append("\":").append(value);
    }

    private void appendJsonNull(StringBuilder builder, String name) {
        builder.append(",\"").append(name).append("\":null");
    }

    private boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    private String safeName(Object value) {
        if (value == null) {
            return null;
        }
        if (value instanceof Enum) {
            return ((Enum<?>) value).name();
        }
        return String.valueOf(value);
    }

    private String escapeJson(String value) {
        return value.replace("\\", "\\\\").replace("\"", "\\\"");
    }

    private void sendPositionUdp(String message) {
        try {
            DatagramSocket socket = positionSocket;
            InetAddress address = rosPcAddress;
            if (socket == null || address == null || socket.isClosed()) {
                return;
            }
            byte[] data = message.getBytes(StandardCharsets.UTF_8);
            DatagramPacket packet = new DatagramPacket(
                    data,
                    data.length,
                    address,
                    POSITION_UDP_PORT
            );
            socket.send(packet);
        } catch (Exception ignored) {
        }
    }

    private synchronized void startVideoTcpBridge() {
        if (videoBridgeStarted) {
            return;
        }

        InetAddress bridgeAddress = rosPcAddress;
        if (bridgeAddress == null) {
            Toast.makeText(getContext(), "DJI ROS bridge address is unknown", Toast.LENGTH_LONG).show();
            return;
        }

        VideoFeeder videoFeeder = VideoFeeder.getInstance();
        if (videoFeeder == null) {
            Toast.makeText(getContext(), "Video feed is null", Toast.LENGTH_LONG).show();
            return;
        }

        primaryVideoFeed = videoFeeder.getPrimaryVideoFeed();
        secondaryVideoFeed = videoFeeder.getSecondaryVideoFeed();
        try {
            transcodedVideoFeed = videoFeeder.provideTranscodedVideoFeed();
        } catch (Exception e) {
            Log.w(TAG, "Transcoded video feed is unavailable", e);
            transcodedVideoFeed = null;
        }

        if (primaryVideoFeed == null && secondaryVideoFeed == null && transcodedVideoFeed == null) {
            Toast.makeText(getContext(), "Video feeds are null", Toast.LENGTH_LONG).show();
            return;
        }

        videoQueue.clear();
        selectedVideoFeedName = null;
        videoBridgeRunning = true;
        videoBridgeStarted = true;

        if (primaryVideoFeed != null) {
            primaryVideoDataListener = createVideoDataListener("primary");
            primaryVideoActiveStatusListener = createVideoActiveStatusListener("primary");
            registerVideoFeed("primary", primaryVideoFeed, primaryVideoDataListener, primaryVideoActiveStatusListener);
            BridgeVideoPreviewView bridgePreviewFeed = getBridgePreviewFeed();
            if (bridgePreviewFeed != null) {
                bridgePreviewFeed.registerLiveVideo(primaryVideoFeed, true);
                scheduleBridgePreviewKeyFrameReset();
                Log.i(TAG, "Registered hidden preview activator on primary video feed");
            }
        }

        if (secondaryVideoFeed != null) {
            secondaryVideoDataListener = createVideoDataListener("secondary");
            secondaryVideoActiveStatusListener = createVideoActiveStatusListener("secondary");
            registerVideoFeed("secondary", secondaryVideoFeed, secondaryVideoDataListener, secondaryVideoActiveStatusListener);
        }

        if (transcodedVideoFeed != null) {
            transcodedVideoDataListener = createVideoDataListener("transcoded");
            transcodedVideoActiveStatusListener = createVideoActiveStatusListener("transcoded");
            registerVideoFeed(
                    "transcoded",
                    transcodedVideoFeed,
                    transcodedVideoDataListener,
                    transcodedVideoActiveStatusListener
            );
        }

        videoExecutor.execute(new Runnable() {
            @Override
            public void run() {
                streamVideoTcp();
            }
        });

        Toast.makeText(
                getContext(),
                "Video TCP bridge started: " + bridgeAddress.getHostAddress(),
                Toast.LENGTH_LONG
        ).show();
        activeVideoBridge = this;
    }

    private synchronized void stopVideoTcpBridge() {
        videoBridgeRunning = false;
        videoBridgeStarted = false;

        if (primaryVideoFeed != null && primaryVideoDataListener != null) {
            primaryVideoFeed.removeVideoDataListener(primaryVideoDataListener);
        }
        if (secondaryVideoFeed != null && secondaryVideoDataListener != null) {
            secondaryVideoFeed.removeVideoDataListener(secondaryVideoDataListener);
        }
        if (transcodedVideoFeed != null && transcodedVideoDataListener != null) {
            transcodedVideoFeed.removeVideoDataListener(transcodedVideoDataListener);
        }
        if (mBridgePreviewFeed != null) {
            mBridgePreviewFeed.unregisterLiveVideo();
        }
        if (primaryVideoFeed != null && primaryVideoActiveStatusListener != null) {
            primaryVideoFeed.removeVideoActiveStatusListener(primaryVideoActiveStatusListener);
        }
        if (secondaryVideoFeed != null && secondaryVideoActiveStatusListener != null) {
            secondaryVideoFeed.removeVideoActiveStatusListener(secondaryVideoActiveStatusListener);
        }
        if (transcodedVideoFeed != null && transcodedVideoActiveStatusListener != null) {
            transcodedVideoFeed.removeVideoActiveStatusListener(transcodedVideoActiveStatusListener);
        }
        primaryVideoFeed = null;
        secondaryVideoFeed = null;
        transcodedVideoFeed = null;
        primaryVideoDataListener = null;
        secondaryVideoDataListener = null;
        transcodedVideoDataListener = null;
        primaryVideoActiveStatusListener = null;
        secondaryVideoActiveStatusListener = null;
        transcodedVideoActiveStatusListener = null;
        selectedVideoFeedName = null;
        videoQueue.clear();
        closeVideoSocket();
        if (activeVideoBridge == this) {
            activeVideoBridge = null;
        }
    }

    private void scheduleBridgePreviewKeyFrameReset() {
        postDelayed(new Runnable() {
            @Override
            public void run() {
                resetBridgePreviewKeyFrame();
            }
        }, 500);
        postDelayed(new Runnable() {
            @Override
            public void run() {
                resetBridgePreviewKeyFrame();
            }
        }, 2000);
    }

    private void resetBridgePreviewKeyFrame() {
        BridgeVideoPreviewView bridgePreviewFeed = getBridgePreviewFeed();
        if (videoBridgeRunning && bridgePreviewFeed != null) {
            bridgePreviewFeed.changeSourceResetKeyFrame();
        }
    }

    private BridgeVideoPreviewView getBridgePreviewFeed() {
        if (mBridgePreviewFeed == null && getContext() instanceof Activity) {
            mBridgePreviewFeed =
                    (BridgeVideoPreviewView) ((Activity) getContext()).findViewById(
                            R.id.video_bridge_preview_feed
                    );
        }
        return mBridgePreviewFeed;
    }

    public static void forwardPreviewVideoPacket(String feedName, byte[] videoBuffer, int size) {
        MainContent bridge = activeVideoBridge;
        if (bridge != null) {
            bridge.enqueueVideoPacket(feedName, videoBuffer, size);
        }
    }

    private VideoFeeder.VideoDataListener createVideoDataListener(final String feedName) {
        return new VideoFeeder.VideoDataListener() {
            @Override
            public void onReceive(byte[] videoBuffer, int size) {
                enqueueVideoPacket(feedName, videoBuffer, size);
            }
        };
    }

    private VideoFeeder.VideoActiveStatusListener createVideoActiveStatusListener(final String feedName) {
        return new VideoFeeder.VideoActiveStatusListener() {
            @Override
            public void onUpdate(boolean isActive) {
                Log.i(TAG, "DJI " + feedName + " video feed active=" + isActive);
            }
        };
    }

    private void registerVideoFeed(
            String feedName,
            VideoFeeder.VideoFeed feed,
            VideoFeeder.VideoDataListener dataListener,
            VideoFeeder.VideoActiveStatusListener activeStatusListener
    ) {
        boolean dataListenerAdded = false;
        try {
            dataListenerAdded = feed.addVideoDataListener(dataListener, true);
        } catch (Exception e) {
            Log.w(TAG, "DJI " + feedName + " video feed raw listener registration failed", e);
        }

        if (!dataListenerAdded) {
            try {
                dataListenerAdded = feed.addVideoDataListener(dataListener);
            } catch (Exception e) {
                Log.w(TAG, "DJI " + feedName + " video feed listener registration failed", e);
            }
        }

        boolean activeListenerAdded = false;
        try {
            activeListenerAdded = feed.addVideoActiveStatusListener(activeStatusListener);
        } catch (Exception e) {
            Log.w(TAG, "DJI " + feedName + " video active listener registration failed", e);
        }

        String sourceName = "unknown";
        try {
            sourceName = String.valueOf(feed.getVideoSource());
        } catch (Exception e) {
            Log.w(TAG, "DJI " + feedName + " video source read failed", e);
        }

        Log.i(
                TAG,
                "DJI " + feedName + " video feed registered: "
                        + "dataListener=" + dataListenerAdded
                        + ", activeListener=" + activeListenerAdded
                        + ", source=" + sourceName
        );
    }

    private void enqueueVideoPacket(String feedName, byte[] videoBuffer, int size) {
        if (!videoBridgeRunning || videoBuffer == null || size <= 0) {
            return;
        }

        String selectedFeed = selectedVideoFeedName;
        if (selectedFeed == null) {
            synchronized (this) {
                if (selectedVideoFeedName == null) {
                    selectedVideoFeedName = feedName;
                    Log.i(TAG, "DJI video bridge selected " + feedName + " feed");
                    showBridgeToast("Video feed selected: " + feedName);
                }
                selectedFeed = selectedVideoFeedName;
            }
        }

        if (feedName.startsWith("preview-") && !selectedFeed.startsWith("preview-")) {
            synchronized (this) {
                selectedVideoFeedName = feedName;
                selectedFeed = feedName;
                Log.i(TAG, "DJI video bridge switched to " + feedName + " feed");
                showBridgeToast("Video feed selected: " + feedName);
            }
        }

        if (!feedName.equals(selectedFeed)) {
            return;
        }

        byte[] packet = Arrays.copyOf(videoBuffer, size);
        if (!videoQueue.offer(packet)) {
            videoQueue.poll();
            videoQueue.offer(packet);
        }
    }

    private void streamVideoTcp() {
        while (videoBridgeRunning) {
            Socket socket = null;
            try {
                InetAddress bridgeAddress = rosPcAddress;
                if (bridgeAddress == null) {
                    sleepQuietly(1000);
                    continue;
                }

                socket = new Socket();
                socket.connect(new InetSocketAddress(bridgeAddress, VIDEO_TCP_PORT), VIDEO_CONNECT_TIMEOUT_MS);
                socket.setTcpNoDelay(true);
                videoSocket = socket;

                post(new Runnable() {
                    @Override
                    public void run() {
                        Toast.makeText(getContext(), "Video TCP bridge connected", Toast.LENGTH_LONG).show();
                    }
                });

                DataOutputStream outputStream =
                        new DataOutputStream(new BufferedOutputStream(socket.getOutputStream()));
                while (videoBridgeRunning && !socket.isClosed()) {
                    byte[] packet = videoQueue.poll(500, TimeUnit.MILLISECONDS);
                    if (packet == null) {
                        continue;
                    }

                    outputStream.writeInt(packet.length);
                    outputStream.write(packet);
                    outputStream.flush();
                }
            } catch (Exception e) {
                if (videoBridgeRunning) {
                    Log.w(TAG, "Video TCP bridge error", e);
                    sleepQuietly(1000);
                }
            } finally {
                closeSocket(socket);
                if (videoSocket == socket) {
                    videoSocket = null;
                }
            }
        }
    }

    private void closeVideoSocket() {
        closeSocket(videoSocket);
        videoSocket = null;
    }

    private void closeSocket(Socket socket) {
        if (socket == null) {
            return;
        }

        try {
            socket.close();
        } catch (Exception ignored) {
        }
    }

    private void sleepQuietly(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
