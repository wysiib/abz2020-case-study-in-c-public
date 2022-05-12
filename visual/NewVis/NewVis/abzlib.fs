module abzlib

open System

open System.Runtime.InteropServices
open FSharp.Core.LanguagePrimitives

module abznative =
    (*
    [<StructLayout(LayoutKind.Explicit)>]
    type scs_state =
        struct
            [<FieldOffset(0)>] val mode:int
            [<FieldOffset(0)>] val cruise_control_active:bool
            [<FieldOffset(0)>] val desired_speed:int
            [<FieldOffset(0)>] val has_previous_desired_speed:bool
            [<FieldOffset(0)>] val previous_desired_speed:int
            [<FieldOffset(0)>] val target_speed:int
            [<FieldOffset(0)>] val current_speed:int
            [<FieldOffset(0)>] val acceleration:int
            [<FieldOffset(0)>] val lever_pos:int
            [<FieldOffset(0)>] val lever_prev_pos:int
            [<FieldOffset(0)>] val lever_last_tic:nativeint
            [<FieldOffset(0)>] val lever_release_processed:bool
            [<FieldOffset(0)>] val lever_continuous:bool
            [<FieldOffset(0)>] val vehicle_speed_infront:int
            [<FieldOffset(0)>] val vehicle_acceleration_infront:int
            [<FieldOffset(0)>] val safety_dist:nativeint
            [<FieldOffset(0)>] val safety_dist_time:int
            [<FieldOffset(0)>] val visual_warning_on:bool
            [<FieldOffset(0)>] val acoustic_warning:int
            [<FieldOffset(0)>] val brake_assistant_available:bool
            [<FieldOffset(0)>] val brake_pressure:int
            [<FieldOffset(0)>] val brake_warning_playing:bool
        end
        *)
        
    type scs_state =
        struct
            val mode:int
            val cruise_control_active:bool
            val desired_speed:int
            val has_previous_desired_speed:bool
            val previous_desired_speed:int
            val target_speed:int
            val current_speed:int
            val acceleration:int64
            val lever_pos:int
            val lever_prev_pos:int
            val lever_last_tic:uint64
            val lever_release_processed:bool
            val lever_continuous:bool
            val vehicle_speed_infront:int
            val vehicle_acceleration_infront:int64
            val safety_dist:uint64
            val safety_dist_time:int
            val visual_warning_on:bool
            val acoustic_warning:int
            val brake_assistant_available:bool
            val brake_pressure:int
            val brake_warning_playing:bool
        end
        
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void nativehelloworld()
    //sensor-stuff
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern int get_brightness()
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern int get_key_status()
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern bool get_engine_status()
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern bool get_all_doors_closed()
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern bool get_reverse_gear()
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern int get_voltage_battery()
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern int get_steering_angle()
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern bool get_oncoming_traffic()
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern nativeint get_time()
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern int get_camera_state()
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern int get_current_speed() 
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern int get_range_radar_state()
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern int read_range_radar_sensor()
    
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void set_brightness(int)
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void set_key_status(int)
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void set_engine_status(bool)
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void set_all_doors_closed(bool)
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void set_reverse_gear(bool)
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void set_voltage_battery(int)
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void set_steering_angle(int)
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void set_oncoming_traffic(bool)
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void set_time(nativeint)
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void set_camera_state(int)
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void set_sensor_current_speed(int)
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void set_range_radar_state(int)
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void set_range_radar_sensor(int)
    //user-interface-stuff
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void lever_forward()  //pull towards driver
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void lever_backward() //push away from driver
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void lever_release()  //stop pull/push
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void lever_up5()
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void lever_up7()
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void lever_down5()
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void lever_down7()
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void brakePedal(int)
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void toggle_adaptive_cruise_control()
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern scs_state get_scs_state()
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void scs_do_step()
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void set_scs_mode(int)
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void set_cruise_control(bool)
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void brake_assistant_available(bool)
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void set_vehicle_speed_infront(int)
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern int get_vehicle_speed_infront()
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern int get_brake_pressure()
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern bool get_brake_assistant_available()
    [<DllImport(@"abz.dll", CallingConvention = CallingConvention.Cdecl)>]
    extern void init_system(int, bool, int,bool,bool);

printfn "Hello from F#"

abznative.nativehelloworld()

let init_system () =
    abznative.init_system(0,false,3,false,false)

type brightness = float
type brightnessConstants =
    | min = 0
    | max = 100000

type keyState = 
    | NoKeyInserted = 0
    | KeyInserted = 1
    | KeyInIgnitionOnPosition = 2

type voltage = float
type voltageConstants = 
    | min = 0
    | max = 500

type steeringAngle = float
type steeringAngleConstants =
    | calibrating = 0
    | hard_left_max = 1
    // resolution: 1 degree
    | hard_left_min = 410
    | soft_left_max = 411
    // resolution: 0.1 degree
    | soft_left_min = 510
    | neutral_maxl = 511
    | neutral = 512
    | neutral_maxr = 513
    | soft_right_min = 514
    // resolution: 0.1 degree
    | soft_right_max = 613
    | hard_right_min = 614
    // resolution: 1 degree
    | hard_right_max = 1022

type sensorState = 
    | Ready = 0
    | Dirty = 1
    | NotReady = 2

type vehicleSpeed = float //resolution 0.1km/h
type vehicleSpeedConstants =
    | min = 0
    | max = 5000

type rangeRadar = float
type rangeRadarConstants =
    | nothing_detected = 0
    | distance_min = 1
    | distance_max = 200
    | faulty = 255

let enumToStr (v:'a when 'a : enum<int>) =
    sprintf "%A" (typedefof<'a>.GetEnumName v)

module sensors =
    //getters

    //no conversion needed
    let get_engine_status = abznative.get_engine_status
    let get_all_doors_closed = abznative.get_all_doors_closed
    let get_reverse_gear = abznative.get_reverse_gear
    let get_oncoming_traffic = abznative.get_oncoming_traffic
    let get_time = abznative.get_time

    //only added type info
    let get_brightness:unit->brightness = 
        abznative.get_brightness>>float

    let get_key_status:unit->keyState =
        abznative.get_key_status>>enum

    let get_camera_state:unit->sensorState =
        abznative.get_camera_state>>enum

    let get_range_radar_state:unit->sensorState =
        abznative.get_range_radar_state>>enum

    //convert weird stuff like steering angles
    let get_voltage_battery:unit->voltage =
        abznative.get_voltage_battery>>float>>(*) 0.1
    
    let toRadian d = d * Math.PI / 180.0
    let toDegree r = (r * 180.0 ) / Math.PI

    let get_steering_angle_degree () : float =
        let raw_val = abznative.get_steering_angle()
        match raw_val with
            |v when v >= int steeringAngleConstants.hard_left_max &&
                    v <= int steeringAngleConstants.hard_left_min
                    -> float <| -10+v-int steeringAngleConstants.hard_left_min
            |v when v >= int steeringAngleConstants.soft_left_max &&
                    v <= int steeringAngleConstants.soft_left_min
                    -> 0.1 * (float <| v-int steeringAngleConstants.soft_left_min)
            |v when v >= int steeringAngleConstants.neutral_maxl &&
                    v <= int steeringAngleConstants.neutral_maxr
                    -> 0.0
            |v when v >= int steeringAngleConstants.soft_right_min &&
                    v <= int steeringAngleConstants.soft_right_max
                    -> 0.1 * (float <| v-int steeringAngleConstants.soft_right_min)
            |v when v >= int steeringAngleConstants.hard_right_min &&
                    v <= int steeringAngleConstants.hard_right_max
                    -> float <| 10+v-int steeringAngleConstants.hard_right_min
            |_ -> nan

    let get_steering_angle_radian () : steeringAngle =
        () |> get_steering_angle_degree |> toRadian

    let get_current_speed () : vehicleSpeed = 
        0.1 * float (abznative.get_current_speed())

    let read_range_radar_sensor () : rangeRadar =
        float (abznative.read_range_radar_sensor())

    //setters
    let set_brightness (v:brightness) =
        abznative.set_brightness(int (round v))
    let set_key_status (v:keyState) =
        abznative.set_key_status(int v)
    let set_engine_status = abznative.set_engine_status
    let set_all_doors_closed = abznative.set_all_doors_closed
    let set_reverse_gear = abznative.set_reverse_gear
    let set_voltage_battery (v:voltage) =
        abznative.set_voltage_battery <| int (10.0 * float v)

    let set_steering_angle_degree (v:float) = 
        
        let valueToSet = 
            match v with
                |v when v < -10.0
                        -> (float <| int steeringAngleConstants.hard_left_min)
                           + v + 10.0
                |v when v >= -10.0 && v <= - 0.1
                        -> (float <| int steeringAngleConstants.soft_left_min)
                           + v * 10.0
                |v when v > -0.1 && v < 0.1
                        -> steeringAngleConstants.neutral |> int |> float
                |v when v >= 0.1 && v <=10.0
                        -> (float <| int steeringAngleConstants.soft_right_min)
                           + v * 10.0
                |v when v > 10.0
                        -> (float <| int steeringAngleConstants.hard_right_min)
                           + v - 10.0
                |_ -> int steeringAngleConstants.calibrating
        let passedval = valueToSet |> round |> int
        let clamped = if passedval > int steeringAngleConstants.hard_right_max 
                        then int steeringAngleConstants.hard_right_max
                      elif passedval < int steeringAngleConstants.hard_left_max
                        then int steeringAngleConstants.hard_left_max
                      else passedval
        abznative.set_steering_angle(clamped)

    let set_steering_angle_radian:steeringAngle->unit =
        toDegree >> round >> set_steering_angle_degree

    let set_oncoming_traffic = abznative.set_oncoming_traffic
    let set_time = abznative.set_time
    let set_camera_state:sensorState->unit =
        int >> abznative.set_camera_state
    let set_sensor_current_speed:vehicleSpeed->unit =
        (*) 10.0 >> round >> int >> abznative.set_sensor_current_speed
    let set_range_radar_state:sensorState->unit =
        int >> abznative.set_range_radar_state
    let set_range_radar_sensor:rangeRadar->unit = 
        round >> int >> abznative.set_range_radar_sensor


    let getSensorStrings () =
        [     
         ("Key Status:",           sprintf "%s" (enumToStr <| get_key_status()));
         ("All doors closed?",     sprintf "%b" (get_all_doors_closed()));
         ("Engine on?",            sprintf "%b" (get_engine_status()));
         ("Reverse Gear?",         sprintf "%b" (get_reverse_gear()));
         ("Current Speed:",        sprintf "%.2f km/h" (get_current_speed()));
         ("Steering Wheel angle:", sprintf "%.2f degree" (get_steering_angle_degree()));
         ("Battery Voltage:",      sprintf "%.2f volt" (get_voltage_battery()));
         ("Brightness:",           sprintf "%.2f lumen" (get_brightness()));
         ("Camera Status:",        sprintf "%s" (enumToStr <| get_camera_state()));
         ("Oncoming Traffic?",     sprintf "%b" (get_oncoming_traffic()));
         ("Radar State:",          sprintf "%s" (enumToStr <| get_range_radar_state()));
         ("Radar distance:",       sprintf "%.2f m" (read_range_radar_sensor()));
         ("Car in front speed:",   sprintf "%.2f km/h" (float (abznative.get_vehicle_speed_infront()) * 0.1));
         ("Current Time:",         sprintf "%d ms" (get_time()));
         ("SCS active?:",          sprintf "%b" (abznative.get_scs_state().cruise_control_active));
         ("SCS mode:",             sprintf "%s" (if abznative.get_scs_state().mode = 1 then "normal" else "adaptive"));
         ("Brake assist avail?",   sprintf "%b" (abznative.get_brake_assistant_available()));
         ("braking pressure:",     sprintf "%i %%" (abznative.get_scs_state().brake_pressure));

        ]
    


module scs_ui = 
    let lever_forward = abznative.lever_forward
    let lever_backward = abznative.lever_backward
    let lever_release  = abznative.lever_release
    let lever_up5 = abznative.lever_up5
    let lever_up7 = abznative.lever_up7
    let lever_down5 = abznative.lever_down5
    let lever_down7 = abznative.lever_down7
    let brakePedal() = abznative.brakePedal 42 // value doesn't even matter :)
    let gasPedal() = () // gaspedal has no impl :O
