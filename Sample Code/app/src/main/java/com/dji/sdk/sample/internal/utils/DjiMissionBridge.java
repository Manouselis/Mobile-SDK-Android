package com.dji.sdk.sample.internal.utils;

import android.content.Context;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.widget.Toast;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import dji.common.error.DJIError;
import dji.common.mission.waypoint.Waypoint;
import dji.common.mission.waypoint.WaypointAction;
import dji.common.mission.waypoint.WaypointActionType;
import dji.common.mission.waypoint.WaypointMission;
import dji.common.mission.waypoint.WaypointMissionDownloadEvent;
import dji.common.mission.waypoint.WaypointMissionExecutionEvent;
import dji.common.mission.waypoint.WaypointMissionFinishedAction;
import dji.common.mission.waypoint.WaypointMissionFlightPathMode;
import dji.common.mission.waypoint.WaypointMissionGotoWaypointMode;
import dji.common.mission.waypoint.WaypointMissionHeadingMode;
import dji.common.mission.waypoint.WaypointMissionState;
import dji.common.mission.waypoint.WaypointMissionUploadEvent;
import dji.common.mission.waypoint.WaypointTurnMode;
import dji.common.model.LocationCoordinate2D;
import dji.common.util.CommonCallbacks;
import dji.sdk.mission.MissionControl;
import dji.sdk.mission.waypoint.WaypointMissionOperator;
import dji.sdk.mission.waypoint.WaypointMissionOperatorListener;

/**
 * TCP mission bridge for the ROS-side MissionPlan JSON and DJI MSDK4 WaypointMissionOperator.
 */
public class DjiMissionBridge {
    private static final String TAG = DjiMissionBridge.class.getName();
    private static final int CONNECT_TIMEOUT_MS = 3000;
    private static final long RECONNECT_DELAY_MS = 2000;
    private static final float DEFAULT_AUTO_SPEED_MPS = 2.0f;
    private static final float DEFAULT_MAX_SPEED_MPS = 5.0f;
    private static final int MAX_WAYPOINT_ACTIONS = 15;

    private final Context context;
    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private final ExecutorService executor = Executors.newSingleThreadExecutor();
    private final InetAddress rosPcAddress;
    private final int missionTcpPort;

    private volatile boolean running = false;
    private volatile Socket socket;
    private volatile BufferedWriter writer;
    private WaypointMissionOperator missionOperator;
    private WaypointMissionOperatorListener listener;
    private WaypointMission currentMission;
    private JSONObject currentPlan;

    public DjiMissionBridge(Context context, InetAddress rosPcAddress, int missionTcpPort) {
        this.context = context.getApplicationContext();
        this.rosPcAddress = rosPcAddress;
        this.missionTcpPort = missionTcpPort;
    }

    public synchronized void start() {
        if (running) {
            return;
        }
        running = true;
        setupMissionOperator();
        executor.execute(new Runnable() {
            @Override
            public void run() {
                runClientLoop();
            }
        });
    }

    public synchronized void stop() {
        running = false;
        closeSocket();
        tearDownMissionOperator();
        executor.shutdownNow();
    }

    private void runClientLoop() {
        while (running) {
            try {
                connectAndReadCommands();
            } catch (Exception e) {
                Log.w(TAG, "Mission bridge connection failed", e);
            } finally {
                closeSocket();
            }

            if (running) {
                sleepBeforeReconnect();
            }
        }
    }

    private void connectAndReadCommands() throws Exception {
        Socket newSocket = new Socket();
        newSocket.connect(
                new InetSocketAddress(rosPcAddress, missionTcpPort),
                CONNECT_TIMEOUT_MS
        );
        socket = newSocket;
        writer = new BufferedWriter(new OutputStreamWriter(
                newSocket.getOutputStream(),
                StandardCharsets.UTF_8
        ));

        BufferedReader reader = new BufferedReader(new InputStreamReader(
                newSocket.getInputStream(),
                StandardCharsets.UTF_8
        ));

        sendStatus("connected", "Android MSDK4 mission executor connected");
        sendToast("Mission TCP bridge connected: " + rosPcAddress.getHostAddress());

        String line;
        while (running && (line = reader.readLine()) != null) {
            handleCommandLine(line);
        }
    }

    private void handleCommandLine(String line) {
        try {
            JSONObject command = new JSONObject(line);
            String name = command.optString("command", "").trim().toLowerCase(Locale.US);
            if ("load".equals(name) || "load_mission".equals(name)) {
                loadMission(command);
            } else if ("upload".equals(name)) {
                uploadMission(command);
            } else if ("start".equals(name)) {
                startMission(command);
            } else if ("pause".equals(name)) {
                pauseMission(command);
            } else if ("resume".equals(name)) {
                resumeMission(command);
            } else if ("stop".equals(name)) {
                stopMission(command);
            } else if ("status".equals(name)) {
                sendStatus("status", "Android MSDK4 mission executor is alive");
            } else {
                sendResult(command, false, "Unsupported mission command: " + name);
            }
        } catch (Exception e) {
            sendError("bad_command", e.getMessage());
        }
    }

    private void loadMission(JSONObject command) {
        try {
            JSONObject plan = command.optJSONObject("plan");
            if (plan == null) {
                plan = command.optJSONObject("mission_plan");
            }
            if (plan == null) {
                sendResult(command, false, "Missing MissionPlan JSON");
                return;
            }

            WaypointMission mission = buildWaypointMission(plan);
            WaypointMissionOperator operator = getMissionOperator();
            if (operator == null) {
                sendResult(command, false, "WaypointMissionOperator unavailable");
                return;
            }

            DJIError error = operator.loadMission(mission);
            if (error != null) {
                sendResult(command, false, error.getDescription());
                return;
            }

            currentPlan = plan;
            currentMission = mission;
            sendResult(command, true, "Mission loaded into DJI MSDK4 operator");
            sendStatus("loaded", "Mission loaded");
        } catch (Exception e) {
            sendResult(command, false, "Mission load failed: " + e.getMessage());
        }
    }

    private WaypointMission buildWaypointMission(JSONObject plan) throws Exception {
        JSONArray waypoints = plan.getJSONArray("waypoints");
        if (waypoints.length() < 2) {
            throw new IllegalArgumentException("DJI waypoint missions need at least 2 waypoints");
        }

        WaypointMission.Builder builder = new WaypointMission.Builder();
        float autoSpeed = chooseAutoFlightSpeed(waypoints);
        float maxSpeed = chooseMaxFlightSpeed(waypoints, autoSpeed);

        builder.autoFlightSpeed(autoSpeed);
        builder.maxFlightSpeed(maxSpeed);
        builder.setExitMissionOnRCSignalLostEnabled(true);
        builder.finishedAction(WaypointMissionFinishedAction.NO_ACTION);
        builder.flightPathMode(WaypointMissionFlightPathMode.NORMAL);
        builder.gotoFirstWaypointMode(WaypointMissionGotoWaypointMode.SAFELY);
        builder.headingMode(WaypointMissionHeadingMode.USING_WAYPOINT_HEADING);
        builder.setGimbalPitchRotationEnabled(true);
        builder.repeatTimes(1);

        LocationCoordinate2D pointOfInterest = firstPointOfInterest(waypoints);
        if (pointOfInterest != null) {
            builder.setPointOfInterest(pointOfInterest);
        }

        for (int i = 0; i < waypoints.length(); i++) {
            builder.addWaypoint(buildWaypoint(waypoints.getJSONObject(i)));
        }

        return builder.build();
    }

    private Waypoint buildWaypoint(JSONObject waypointJson) throws Exception {
        double latitude = waypointJson.getDouble("latitude");
        double longitude = waypointJson.getDouble("longitude");
        float altitude = (float) waypointJson.getDouble("altitude_m");

        Waypoint waypoint = new Waypoint(latitude, longitude, altitude);
        if (waypointJson.has("heading_deg") && !waypointJson.isNull("heading_deg")) {
            waypoint.heading = Math.round(normalizeHeadingToDjiRange(waypointJson.getDouble("heading_deg")));
        }
        if (waypointJson.has("gimbal_pitch_deg") && !waypointJson.isNull("gimbal_pitch_deg")) {
            waypoint.gimbalPitch = clamp(
                    (float) waypointJson.getDouble("gimbal_pitch_deg"),
                    -90.0f,
                    30.0f
            );
        }
        if (waypointJson.has("curve_size_m") && !waypointJson.isNull("curve_size_m")) {
            float cornerRadius = (float) waypointJson.getDouble("curve_size_m");
            if (cornerRadius > 0.0f) {
                waypoint.cornerRadiusInMeters = Math.max(0.2f, cornerRadius);
            }
        }
        if (waypointJson.has("rotation_dir") && !waypointJson.isNull("rotation_dir")) {
            waypoint.turnMode = waypointJson.getInt("rotation_dir") == 1
                    ? WaypointTurnMode.COUNTER_CLOCKWISE
                    : WaypointTurnMode.CLOCKWISE;
        }
        if (waypointJson.has("speed_mps") && !waypointJson.isNull("speed_mps")) {
            waypoint.speed = clamp((float) waypointJson.getDouble("speed_mps"), 0.0f, 15.0f);
        }
        if (waypointJson.has("photo_time_interval") && !waypointJson.isNull("photo_time_interval")) {
            float interval = (float) waypointJson.getDouble("photo_time_interval");
            if (interval > 0.0f) {
                waypoint.shootPhotoTimeInterval = interval;
            }
        }
        if (waypointJson.has("photo_distance_interval") && !waypointJson.isNull("photo_distance_interval")) {
            float interval = (float) waypointJson.getDouble("photo_distance_interval");
            if (interval > 0.0f) {
                waypoint.shootPhotoDistanceInterval = interval;
            }
        }

        JSONArray actions = waypointJson.optJSONArray("actions");
        if (actions != null) {
            for (int i = 0; i < actions.length() && i < MAX_WAYPOINT_ACTIONS; i++) {
                WaypointAction action = buildWaypointAction(actions.getJSONObject(i));
                if (action != null) {
                    waypoint.addAction(action);
                }
            }
        }
        return waypoint;
    }

    private WaypointAction buildWaypointAction(JSONObject actionJson) {
        int litchiType = actionJson.optInt("type", -1);
        int param = actionJson.isNull("param")
                ? 0
                : (int) Math.round(actionJson.optDouble("param", 0.0));

        switch (litchiType) {
            case 0:
                return new WaypointAction(
                        WaypointActionType.STAY,
                        clampInt(param, 0, 32767)
                );
            case 1:
                return new WaypointAction(WaypointActionType.START_TAKE_PHOTO, 0);
            case 2:
                return new WaypointAction(WaypointActionType.START_RECORD, 0);
            case 3:
                return new WaypointAction(WaypointActionType.STOP_RECORD, 0);
            case 4:
                return new WaypointAction(
                        WaypointActionType.ROTATE_AIRCRAFT,
                        Math.round(normalizeHeadingToDjiRange(param))
                );
            case 5:
                return new WaypointAction(
                        WaypointActionType.GIMBAL_PITCH,
                        clampInt(param, -90, 30)
                );
            default:
                Log.w(TAG, "Skipping unsupported Litchi action type: " + litchiType);
                return null;
        }
    }

    private void uploadMission(final JSONObject command) {
        WaypointMissionOperator operator = getMissionOperator();
        if (operator == null || currentMission == null) {
            sendResult(command, false, "Mission must be loaded before upload");
            return;
        }

        operator.uploadMission(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                sendResult(command, error == null, djiErrorMessage(error, "Mission upload started"));
            }
        });
    }

    private void startMission(final JSONObject command) {
        WaypointMissionOperator operator = getMissionOperator();
        if (operator == null || currentMission == null) {
            sendResult(command, false, "Mission must be loaded before start");
            return;
        }

        operator.startMission(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                sendResult(command, error == null, djiErrorMessage(error, "Mission start accepted"));
            }
        });
    }

    private void pauseMission(final JSONObject command) {
        WaypointMissionOperator operator = getMissionOperator();
        if (operator == null) {
            sendResult(command, false, "WaypointMissionOperator unavailable");
            return;
        }

        operator.pauseMission(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                sendResult(command, error == null, djiErrorMessage(error, "Mission pause accepted"));
            }
        });
    }

    private void resumeMission(final JSONObject command) {
        WaypointMissionOperator operator = getMissionOperator();
        if (operator == null) {
            sendResult(command, false, "WaypointMissionOperator unavailable");
            return;
        }

        operator.resumeMission(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                sendResult(command, error == null, djiErrorMessage(error, "Mission resume accepted"));
            }
        });
    }

    private void stopMission(final JSONObject command) {
        WaypointMissionOperator operator = getMissionOperator();
        if (operator == null) {
            sendResult(command, false, "WaypointMissionOperator unavailable");
            return;
        }

        operator.stopMission(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                if (error == null) {
                    currentMission = null;
                }
                sendResult(command, error == null, djiErrorMessage(error, "Mission stop accepted"));
            }
        });
    }

    private void setupMissionOperator() {
        missionOperator = getMissionOperator();
        if (missionOperator == null || listener != null) {
            return;
        }

        listener = new WaypointMissionOperatorListener() {
            @Override
            public void onDownloadUpdate(@NonNull WaypointMissionDownloadEvent event) {
                sendEvent("download_update", null, event);
            }

            @Override
            public void onUploadUpdate(@NonNull WaypointMissionUploadEvent event) {
                sendEvent("upload_update", event, null);
            }

            @Override
            public void onExecutionUpdate(@NonNull WaypointMissionExecutionEvent event) {
                sendExecutionEvent("execution_update", event, null);
            }

            @Override
            public void onExecutionStart() {
                sendStatus("executing", "Mission execution started");
            }

            @Override
            public void onExecutionFinish(@Nullable DJIError error) {
                sendStatus(error == null ? "finished" : "failed", djiErrorMessage(error, "Mission finished"));
            }
        };
        missionOperator.addListener(listener);
    }

    private void tearDownMissionOperator() {
        if (missionOperator != null && listener != null) {
            missionOperator.removeListener(listener);
        }
        listener = null;
    }

    private WaypointMissionOperator getMissionOperator() {
        if (missionOperator == null && MissionControl.getInstance() != null) {
            missionOperator = MissionControl.getInstance().getWaypointMissionOperator();
        }
        return missionOperator;
    }

    private void sendEvent(String type, WaypointMissionUploadEvent uploadEvent, WaypointMissionDownloadEvent downloadEvent) {
        try {
            JSONObject event = baseMessage("event");
            event.put("event", type);
            addOperatorState(event);
            if (uploadEvent != null && uploadEvent.getProgress() != null) {
                event.put("uploaded_waypoint_index", uploadEvent.getProgress().uploadedWaypointIndex);
                event.put("is_summary_uploaded", uploadEvent.getProgress().isSummaryUploaded);
            }
            if (downloadEvent != null && downloadEvent.getProgress() != null) {
                event.put("downloaded_waypoint_index", downloadEvent.getProgress().downloadedWaypointIndex);
                event.put("is_summary_downloaded", downloadEvent.getProgress().isSummaryDownloaded);
            }
            sendJson(event);
        } catch (Exception e) {
            Log.d(TAG, "Mission event JSON failed", e);
        }
    }

    private void sendExecutionEvent(String type, WaypointMissionExecutionEvent executionEvent, DJIError error) {
        try {
            JSONObject event = baseMessage("event");
            event.put("event", type);
            addOperatorState(event);
            if (executionEvent != null && executionEvent.getProgress() != null) {
                event.put("target_waypoint_index", executionEvent.getProgress().targetWaypointIndex);
                event.put("is_waypoint_reached", executionEvent.getProgress().isWaypointReached);
            }
            if (error != null) {
                event.put("success", false);
                event.put("message", error.getDescription());
            }
            sendJson(event);
        } catch (Exception e) {
            Log.d(TAG, "Mission execution event JSON failed", e);
        }
    }

    private void sendResult(JSONObject command, boolean success, String message) {
        try {
            JSONObject result = baseMessage("result");
            result.put("command", command.optString("command", ""));
            result.put("id", command.optString("id", ""));
            result.put("success", success);
            result.put("message", message);
            addOperatorState(result);
            sendJson(result);
        } catch (Exception e) {
            Log.d(TAG, "Mission result JSON failed", e);
        }
    }

    private void sendStatus(String state, String message) {
        try {
            JSONObject status = baseMessage("status");
            status.put("state", state);
            status.put("message", message);
            status.put("has_loaded_mission", currentMission != null);
            if (currentPlan != null) {
                status.put("waypoint_count", currentPlan.optJSONArray("waypoints") == null
                        ? 0
                        : currentPlan.optJSONArray("waypoints").length());
            }
            addOperatorState(status);
            sendJson(status);
        } catch (Exception e) {
            Log.d(TAG, "Mission status JSON failed", e);
        }
    }

    private void sendError(String code, String message) {
        try {
            JSONObject error = baseMessage("error");
            error.put("code", code);
            error.put("message", message);
            addOperatorState(error);
            sendJson(error);
        } catch (Exception e) {
            Log.d(TAG, "Mission error JSON failed", e);
        }
    }

    private JSONObject baseMessage(String type) throws Exception {
        JSONObject message = new JSONObject();
        message.put("t", System.currentTimeMillis());
        message.put("source", "android_mission_executor");
        message.put("type", type);
        return message;
    }

    private void addOperatorState(JSONObject message) throws Exception {
        WaypointMissionOperator operator = getMissionOperator();
        WaypointMissionState state = operator == null ? null : operator.getCurrentState();
        message.put("operator_state", state == null ? JSONObject.NULL : state.getName());
    }

    private synchronized void sendJson(JSONObject message) {
        try {
            BufferedWriter currentWriter = writer;
            if (currentWriter == null) {
                return;
            }
            currentWriter.write(message.toString());
            currentWriter.write('\n');
            currentWriter.flush();
        } catch (Exception e) {
            Log.d(TAG, "Mission bridge send failed", e);
        }
    }

    private LocationCoordinate2D firstPointOfInterest(JSONArray waypoints) {
        for (int i = 0; i < waypoints.length(); i++) {
            JSONObject waypoint = waypoints.optJSONObject(i);
            if (waypoint == null) {
                continue;
            }
            JSONObject poi = waypoint.optJSONObject("poi");
            if (poi != null && !poi.isNull("latitude") && !poi.isNull("longitude")) {
                return new LocationCoordinate2D(
                        poi.optDouble("latitude"),
                        poi.optDouble("longitude")
                );
            }
        }
        return null;
    }

    private float chooseAutoFlightSpeed(JSONArray waypoints) {
        for (int i = 0; i < waypoints.length(); i++) {
            JSONObject waypoint = waypoints.optJSONObject(i);
            if (waypoint == null || waypoint.isNull("speed_mps")) {
                continue;
            }
            double speed = waypoint.optDouble("speed_mps", DEFAULT_AUTO_SPEED_MPS);
            if (speed > 0.0) {
                return clamp((float) speed, 1.0f, 15.0f);
            }
        }
        return DEFAULT_AUTO_SPEED_MPS;
    }

    private float chooseMaxFlightSpeed(JSONArray waypoints, float autoSpeed) {
        float maxSpeed = Math.max(DEFAULT_MAX_SPEED_MPS, autoSpeed);
        for (int i = 0; i < waypoints.length(); i++) {
            JSONObject waypoint = waypoints.optJSONObject(i);
            if (waypoint != null && !waypoint.isNull("speed_mps")) {
                maxSpeed = Math.max(maxSpeed, (float) waypoint.optDouble("speed_mps", maxSpeed));
            }
        }
        return clamp(maxSpeed, 2.0f, 15.0f);
    }

    private float normalizeHeadingToDjiRange(double heading) {
        double normalized = heading % 360.0;
        if (normalized > 180.0) {
            normalized -= 360.0;
        } else if (normalized < -180.0) {
            normalized += 360.0;
        }
        return (float) normalized;
    }

    private float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
    }

    private int clampInt(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }

    private String djiErrorMessage(DJIError error, String successMessage) {
        return error == null ? successMessage : error.getDescription();
    }

    private void closeSocket() {
        try {
            if (socket != null) {
                socket.close();
            }
        } catch (Exception ignored) {
        }
        socket = null;
        writer = null;
    }

    private void sleepBeforeReconnect() {
        try {
            Thread.sleep(RECONNECT_DELAY_MS);
        } catch (InterruptedException ignored) {
            Thread.currentThread().interrupt();
        }
    }

    private void sendToast(final String message) {
        mainHandler.post(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(context, message, Toast.LENGTH_LONG).show();
            }
        });
    }
}
