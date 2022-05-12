open System

open abzlib

open Raylib_cs
open type Raylib_cs.Raylib
open FSharp.Reflection
open System.Numerics

let contains (s1:string) (s2:string) : bool = 
    s1.Contains s2

printfn "Can you spell dysfunctional without fun? %A" (contains "dysfunctional" "fun" |> not)

let drawloop initstate updatefn drawfn =
    let rec drawimpl state =
        if not <| WindowShouldClose() then 
            BeginDrawing()
            ClearBackground Color.WHITE
            drawfn state
            EndDrawing()
            let newstate = updatefn state
            drawimpl newstate
        else
            CloseWindow()

    drawimpl initstate

let toRadian d = d * Math.PI / 180.0
let toDegree r = (r * 180.0 ) / Math.PI

let width=1200
let height=800

InitWindow (width,height,"ABZ in C driven with F# + Raylib 🤯")
SetTargetFPS 60

let standard_steering_aggressiveness = 5.0 //1.0 = normal

type car={|x:double;
           y:double;
           rotation:double;
           speed:double;
           rotation_circle:Vector2*float;
           steeringAngle:float;
           steering_aggressiveness:float
           |}

let random = System.Random()

let initcar:car = {|
    x=0;y=0;rotation=0;
    rotation_circle=(Vector2(0f,0f),0.0);
    steeringAngle=1.0;
    speed=0.0;
    steering_aggressiveness = standard_steering_aggressiveness
|}

let make_dummy_car x y :car =
    {|x=x;y=y;
      rotation=random.NextDouble();
      rotation_circle=(Vector2(0f,0f),0.0);
      steeringAngle=1.0;
      speed=0.5-(0.05+random.NextDouble()*0.2);
      steering_aggressiveness = 1.5+random.NextDouble()*3.0
    |}

type visstate = {|
    abz_car:car;
    ai_cars:car list;
    time:double;
    gaspedal:bool;
    brakepedal:bool;
    speed_history:vehicleSpeed list
    distance_history:rangeRadar list
    steering_history:steeringAngle list
|}

let n_ai_cars = 1
let ai_cars = Seq.initInfinite (fun i -> make_dummy_car (sin i) (cos i)) |> Seq.take n_ai_cars |> Seq.toList

init_system()
sensors.set_sensor_current_speed(initcar.speed*100.0)
sensors.set_time(0)

let firacode = LoadFont "FiraCode-Regular.ttf"
SetTextureFilter(firacode.texture, TextureFilter.TEXTURE_FILTER_TRILINEAR);
printf "firacode base font size is: %d\n" firacode.baseSize

let drawFira (text:string) x y =
    DrawTextEx(firacode,text,Numerics.Vector2(x,y),0.5f*(float32 firacode.baseSize),1.0f,Color.DARKGRAY)

let drawStringList x fromy (lines:string list) =
    let rec impl cury (remaining:string list) =
        if not <| List.isEmpty remaining then
            drawFira remaining.Head x cury
            impl (cury+13f) remaining.Tail
    impl fromy lines

let drawtextual (state:car) = 
    drawFira "Hello from F#" 20f 20f
    let sensorstrings = sensors.getSensorStrings()
    let xoff = 10f
    let yoff = 50f
    drawStringList xoff        yoff (List.map fst sensorstrings)
    drawStringList (xoff+200f) yoff (List.map snd sensorstrings)
    drawFira (sprintf "car rotation: %.2f" (float state.rotation)) 220f 20f

let car = LoadModel "models/cars/NormalCar2.obj"


let origin = Numerics.Vector3(0f,0f,0f)
let up = Numerics.Vector3(0f,1f,0f)
let camera = Camera3D (Numerics.Vector3(16f,12f,16f),origin,up,45f,CameraProjection.CAMERA_PERSPECTIVE)

let wheel_pic = LoadTexture("pics/steering_wheel.png")
SetTextureFilter(wheel_pic, TextureFilter.TEXTURE_FILTER_TRILINEAR);

let pedal_pic = LoadTexture("pics/pedal.png")
SetTextureFilter(pedal_pic, TextureFilter.TEXTURE_FILTER_TRILINEAR);

SetCameraMode(camera,CameraMode.CAMERA_FREE)

sensors.set_steering_angle_degree 180

printf "steering angle: %.2f\n" (float <| sensors.get_steering_angle_degree())
//according to spec the wheel can rotate from -418° (or 420°) to 418° (or 420°). So 840°. 
//That is about 2+1/3 full rotations. Seems a bit low.. 

//lets pick a reasonably low steering ratio since we have so little roations
//wikipedia says 12:1 is ok.

let steering_ratio = 1.0/12.0

let car_length = abs (GetModelBoundingBox(car).max.X - GetModelBoundingBox(car).min.X)


let tuple_numvec t =
    Numerics.Vector2(float32 <| fst t,float32 <| snd t)

let polar_to_cartesian angle r =
    (r*cos(angle),r*sin(angle))

let unit_vec angle = 
    tuple_numvec <| polar_to_cartesian angle 1.0

//car will follow a lemniscate by the means of lorentz-attractor like steering

let attractorset1 = [Vector2(-20f,0f);Vector2(20f,0f)]
let attractorset2 = [Vector2(-10f,0f);Vector2(00f,0f);Vector2(10f,0f);
                     Vector2(15f,5f);Vector2(20f,10f);Vector2(20f,20f)]
let attractorset3_n = 10
let attractorset3 = List.map ((fun a->polar_to_cartesian a 25.0)>>tuple_numvec) [for i in 1 .. attractorset3_n -> double i * (Math.PI*2.0/double attractorset3_n) ]
let attractors = attractorset3
let closest_point (l:Vector2 list) (p:Vector2) =
    l 
    |> List.sortBy (fun v -> Vector2.Distance(p,v))
    |> List.head

let closest_attractor = closest_point attractors

let steer_change_from_point_old (p:Vector2) (rotation:double) current_steering steering_aggressiveness =
    let attractor = closest_attractor p
    let distance = Vector2.Distance(p,attractor)
    let heading = unit_vec (rotation-Math.PI)
    let normal = unit_vec (rotation-(Math.PI/2.0))
    let diff = Vector2.Normalize(p-attractor)
    let steercorr = 0.1*current_steering
    float (Vector2.Dot(diff,normal)) * steering_aggressiveness * (1.0+((distance |> float |> log))) - steercorr


let steer_change_from_point (p:Vector2) (attractors:Vector2 list) (rotation:double) current_steering steering_aggressiveness =
    let attractor = closest_point attractors p
    let distance = Vector2.Distance(p,attractor)
    let heading = unit_vec (rotation-Math.PI)
    let normal = unit_vec (rotation-(Math.PI/2.0))
    let diff = Vector2.Normalize(p-attractor)
    let steercorr = 0.1*current_steering
    float (Vector2.Dot(diff,normal)) * steering_aggressiveness * (1.0+((distance |> float |> log))) - steercorr


let get_wheelrotation () = 
    ((sensors.get_steering_angle_radian())*steering_ratio)

let get_car_rotation_circle carpos car_rotation len wheel_rotation =
    if(abs wheel_rotation < 0.0001) then
        let phi = car_rotation+Math.PI/2.0
        ((unit_vec car_rotation),infinity)
    else
        let m = tuple_numvec carpos
        let phi = car_rotation+Math.PI/2.0
        let f = m+((unit_vec car_rotation)*(float32 len/2f))
        let b = m-((unit_vec car_rotation)*(float32 len/2f))
        let r = len/(sin wheel_rotation)
        let c = m+((unit_vec phi)*float32 r)
        (c,r)

let circle_movement (c:Vector2) r (oldpos:Vector2) arclen =
    if r=infinity then
        (oldpos+c*float32 arclen,nan)
    else
        let alpha = arclen/r
        let diff = if alpha > 0.0 then oldpos-c else c-oldpos
        let oldangle = Math.Atan2(float diff.Y, float diff.X)
        let newangle = (double oldangle + alpha)
        let newcircleposorigin = tuple_numvec <| polar_to_cartesian newangle r
        let newcirclepos = c+newcircleposorigin
        let newrotation = (Math.PI/2.0)+(newangle*1.0)
        (newcirclepos,newrotation)


let move_car (old:car) =
    if old.speed<0.001 then old else
    let steering_angle = old.steeringAngle
    let steering_angle_radian = steering_angle |> toRadian 
    let m = tuple_numvec <| (old.x,old.y)
    let (c,r) = get_car_rotation_circle (old.x,old.y) old.rotation (float car_length) (steering_angle_radian*steering_ratio)
    let m = tuple_numvec (old.x,old.y)
    let (newcarpos,newcircrotation) = circle_movement c r m old.speed
    let newrotation = if Double.IsNaN(newcircrotation) then old.rotation else newcircrotation
    {|old with 
        x=double newcarpos.X;
        y=double newcarpos.Y;
        rotation=newrotation;
        steeringAngle=steering_angle;
        rotation_circle=(c,r);
        |}

let move_abzcar (old:car) =
    let steering_angle = sensors.get_steering_angle_degree()
    let steering_angle_radian = steering_angle |> toRadian 
    let m = tuple_numvec <| (old.x,old.y)
    let (c,r) = get_car_rotation_circle (old.x,old.y) old.rotation (float car_length) (steering_angle_radian*steering_ratio)
    let m = tuple_numvec (old.x,old.y)
    let (newcarpos,newcircrotation) = circle_movement c r m old.speed
    let newrotation = if Double.IsNaN(newcircrotation) then old.rotation else newcircrotation
    {|old with 
        x=double newcarpos.X;
        y=double newcarpos.Y;
        rotation=newrotation;
        steeringAngle=steering_angle;
        rotation_circle=(c,r);
        |}

let getradarinfo (abzcar:car,ai_cars:car list) =
    let carpos = tuple_numvec (abzcar.x,abzcar.y)
    let closest = closest_point (List.map (fun (c:car)->tuple_numvec (c.x,c.y)) ai_cars) (tuple_numvec (abzcar.x,abzcar.y))
    (carpos,closest)

let drawCar color (state:car) =
    let mutable carcopy = car
    carcopy.transform <- Raymath.MatrixRotateY(float32 (state.rotation-Math.PI/2.0))
    
    DrawModel(carcopy,
              Numerics.Vector3(float32 state.x,0f,float32 state.y),
              1f,
              color
              )


let drawCarDiagnosticLines (state:car) =
    let (c,r) = state.rotation_circle
    let m = tuple_numvec (state.x,state.y)

    let (newcirclepos,newrotation) = circle_movement c r m state.speed

    DrawCylinderWires(Numerics.Vector3(c.X,0f,c.Y),
                      float32 r, float32 r, 0.01f, 40, Color.DARKBLUE)
    DrawSphere(Numerics.Vector3(c.X,0f,c.Y), 0.1f ,Color.DARKPURPLE)
    DrawSphere(Numerics.Vector3(float32 state.x,0f,float32 state.y), 0.1f ,Color.GREEN)


    let newcarpos = newcirclepos
    let oldcarpos = tuple_numvec (state.x,state.y)
    
    let posdiff = newcarpos-oldcarpos
    let startline = newcarpos-(posdiff*100f)
    let endline = newcarpos+(posdiff*100f)
    DrawLine3D(Numerics.Vector3(startline.X,0f,startline.Y),
               Numerics.Vector3(endline.X,0f,endline.Y), Color.ORANGE);
    
    let rotationlinestart = oldcarpos - ((unit_vec newrotation)*100f)
    let rotationlineend = oldcarpos + ((unit_vec newrotation)*100f)
    DrawLine3D(Numerics.Vector3(rotationlinestart.X,0f,rotationlinestart.Y),
               Numerics.Vector3(rotationlineend.X,0f,rotationlineend.Y), Color.MAGENTA);
    DrawSphere(Numerics.Vector3(newcirclepos.X,0f,newcirclepos.Y), 0.05f ,Color.RED)
    
let draw_radarline (abzcar:car,ai_cars:car list) =
    let (a,b) = getradarinfo (abzcar,ai_cars)
    DrawLine3D(Numerics.Vector3(a.X,1f,a.Y),
               Numerics.Vector3(b.X,1f,b.Y), Color.RED);

    
    
let draw_attractors () =
    ignore <| List.map 
                (fun (attractor:Vector2) -> 
                    DrawCylinder(Vector3(attractor.X,-0.02f,attractor.Y),
                                 0.3f, 0.3f, 0.04f, 40, Color.DARKBLUE)) 
                attractors



let draw3D (abzcar:car,ai_cars:car list) =
    let camera = Camera3D (Numerics.Vector3(float32 abzcar.x+16f,12f,float32 abzcar.y+16f),
                Vector3(float32 abzcar.x,0f,float32 abzcar.y),up,45f,CameraProjection.CAMERA_PERSPECTIVE)
    
    SetCameraMode(camera,CameraMode.CAMERA_FREE)

    BeginMode3D(camera)

    drawCar Color.GRAY abzcar
    drawCarDiagnosticLines abzcar

    DrawGrid(20, 10.0f)
    ignore <| List.map (drawCar Color.MAROON) ai_cars

    draw_attractors()
    draw_radarline(abzcar,ai_cars)
    EndMode3D()

let draw_plot (values:float list) xoff yoff =
    if values.Length < 2 then () else
    let maxv = List.max values
    let minv = List.min values
    let dist = max (abs(maxv-minv)) (abs(minv-maxv))
    let ysize = 100.0
    let xsize = 0.1

    let points:Vector2 list = 
        values
        |> List.map (fun v->(ysize*(v-minv))/dist)
        |> List.map2 (fun a b -> (a,b)) [for i in 1..values.Length -> xsize * float i]
        |> List.map tuple_numvec
        |> List.map (fun v->v+Vector2(xoff,yoff))

    DrawLineStrip(List.toArray points,points.Length,Color.RED)

let draw_images (state:visstate) =
    let rotation = float32<|sensors.get_steering_angle_degree();
    Raylib_cs.Rlgl.rlPushMatrix();
    Raylib_cs.Rlgl.rlTranslatef(
        (float32 width)-(float32 wheel_pic.width/3f),
        (float32 wheel_pic.height/3f), 
        0f);
    Raylib_cs.Rlgl.rlScalef(0.4f,0.4f,1f);
    Raylib_cs.Rlgl.rlRotatef(rotation,0f,0f,1f);
    Raylib_cs.Rlgl.rlTranslatef(float32 -wheel_pic.width/2f,float32 -wheel_pic.height/2f, 0f);
    
    DrawTexture(wheel_pic,0,0,Color.WHITE)

    Raylib_cs.Rlgl.rlPopMatrix();

    Raylib_cs.Rlgl.rlPushMatrix();
    Raylib_cs.Rlgl.rlTranslatef(
        (float32 width)-(float32 pedal_pic.width/3f),
        450f, 
        0f);

    Raylib_cs.Rlgl.rlScalef(0.3f,0.3f,1f);
    if state.brakepedal then 
        Raylib_cs.Rlgl.rlRotatef(45f,0f,0f,1f);
    else 
        Raylib_cs.Rlgl.rlRotatef(0f,0f,0f,1f);
    Raylib_cs.Rlgl.rlTranslatef(float32 -pedal_pic.width/2f,float32 -pedal_pic.height/2f, 0f);
    
    DrawTexture(pedal_pic,0,0,Color.RED)

    Raylib_cs.Rlgl.rlPopMatrix();

    Raylib_cs.Rlgl.rlPushMatrix();
    Raylib_cs.Rlgl.rlTranslatef(
        (float32 width)-(float32 pedal_pic.width/5f),
        450f, 
        0f);

    Raylib_cs.Rlgl.rlScalef(0.3f,0.3f,1f);
    if state.gaspedal then 
        Raylib_cs.Rlgl.rlRotatef(45f,0f,0f,1f);
    else 
        Raylib_cs.Rlgl.rlRotatef(0f,0f,0f,1f);
    Raylib_cs.Rlgl.rlTranslatef(float32 -pedal_pic.width/2f,float32 -pedal_pic.height/2f, 0f);
    
    DrawTexture(pedal_pic,0,0,Color.RED)

    Raylib_cs.Rlgl.rlPopMatrix();


    //DrawTextureEx(wheel_pic,Vector2(200f,500f),float32<|sensors.get_steering_angle_degree(),0.5f,Color.WHITE)
    

let drawAll (state:visstate) =
    let (abzcar,ai_cars) = (state.abz_car,state.ai_cars)
    draw3D (abzcar,ai_cars)
    drawtextual abzcar
    //draw_plot (List.map sin [for i in 1 .. 50 -> double i * (Math.PI*2.0/double 50)]) 400f 400f
    draw_plot state.distance_history 20f 400f
    draw_plot state.speed_history 20f 550f
    draw_plot state.steering_history 20f 700f
    draw_images state

let autopilot_get_new_steering_angle (car:car) =
    car.steeringAngle+
    (steer_change_from_point 
         (tuple_numvec (car.x,car.y))
         attractors
         car.rotation
         car.steeringAngle
         car.steering_aggressiveness)

let autopilot_follow_get_new_steering_angle (car:car) (car_to_follow:car) =
    car.steeringAngle+
    (steer_change_from_point 
         (tuple_numvec (car.x,car.y))
         [(tuple_numvec (car_to_follow.x,car_to_follow.y))]
         car.rotation
         car.steeringAngle
         car.steering_aggressiveness)


let apply_autopilot (car:car) =
    {|car with steeringAngle = autopilot_get_new_steering_angle car|}

#nowarn "3391"
let handle_input (state:visstate) =
    let (abzcar,ai_cars) = (state.abz_car,state.ai_cars)
    let mutable gaspedal = false;
    let mutable brakepedal = false;

    if IsKeyDown(KeyboardKey.KEY_RIGHT)=true then
        sensors.set_steering_angle_degree(abzcar.steeringAngle+1.0)
    if IsKeyDown(KeyboardKey.KEY_LEFT)=true then
        sensors.set_steering_angle_degree(abzcar.steeringAngle-1.0)
    if IsKeyDown(KeyboardKey.KEY_G)=true then
        scs_ui.gasPedal()
        gaspedal <- true
    if IsKeyDown(KeyboardKey.KEY_B)=true then
        scs_ui.brakePedal()
        brakepedal <- true
    if IsKeyDown(KeyboardKey.KEY_UP)=true then
        let speed = sensors.get_current_speed()
        if IsKeyDown(KeyboardKey.KEY_LEFT_SHIFT)=true then
            sensors.set_sensor_current_speed(speed+0.3)
        else
            sensors.set_sensor_current_speed(speed+0.1)
    if IsKeyDown(KeyboardKey.KEY_DOWN)=true then
        let speed = sensors.get_current_speed()
        if IsKeyDown(KeyboardKey.KEY_LEFT_SHIFT)=true then
            sensors.set_sensor_current_speed(max 0.0 (speed-0.3))
        else 
            sensors.set_sensor_current_speed(max 0.0 (speed-0.1))
    if IsKeyDown(KeyboardKey.KEY_A)=true then
        sensors.set_steering_angle_degree(0.0)
    if IsKeyDown(KeyboardKey.KEY_B)=true then
        sensors.set_steering_angle_degree(-9.0)
    if IsKeyDown(KeyboardKey.KEY_C)=true then
        sensors.set_steering_angle_degree(-360.0)
    if IsKeyDown(KeyboardKey.KEY_P)=true then
        sensors.set_steering_angle_degree(autopilot_get_new_steering_angle abzcar)
    if IsKeyDown(KeyboardKey.KEY_F)=true then
        sensors.set_steering_angle_degree(autopilot_follow_get_new_steering_angle abzcar ai_cars.Head)
    if IsKeyDown(KeyboardKey.KEY_ONE)=true then
        abznative.set_scs_mode(1)
    if IsKeyDown(KeyboardKey.KEY_TWO)=true then
        abznative.set_scs_mode(2)
    if IsKeyDown(KeyboardKey.KEY_N)=true then
        abznative.set_cruise_control(true)
    if IsKeyDown(KeyboardKey.KEY_M)=true then
        abznative.set_cruise_control(false)
    if IsKeyDown(KeyboardKey.KEY_C)=true then
        sensors.set_key_status keyState.NoKeyInserted
    if IsKeyDown(KeyboardKey.KEY_V)=true then
        sensors.set_key_status keyState.KeyInserted
    if IsKeyDown(KeyboardKey.KEY_B)=true then
        sensors.set_engine_status(true)
    if IsKeyDown(KeyboardKey.KEY_S)=true then
        printf "screenshot time :)"

    abznative.scs_do_step()

    {|state with 
        abz_car={|abzcar with 
            steeringAngle = sensors.get_steering_angle_degree();
            speed = sensors.get_current_speed()/100.0;
        |};
        brakepedal = brakepedal
        gaspedal = gaspedal
    |}

let set_sensorstate (state:visstate) =
    let (abzcar,ai_cars) = (state.abz_car,state.ai_cars)

    let (carpos,closest) = getradarinfo (abzcar,ai_cars)
    let closest_car = List.filter (fun (c:car)->float32 c.x=closest.X && float32 c.y=closest.Y) ai_cars |> List.head

    let dist:rangeRadar = Vector2.Distance(carpos,closest) |> float

    sensors.set_range_radar_sensor(dist)
    let newtime = state.time+(1000.0/60.0)
    sensors.set_time(nativeint newtime)

    let scs_before = abznative.get_scs_state()
    let frontvehiclespeed = (closest_car.speed*1000.0) |> round |> int
    abznative.set_vehicle_speed_infront(frontvehiclespeed)
    let scs_after = abznative.get_vehicle_speed_infront()


    {|state with time = newtime+(1000.0/60.0)|}

let updatefn (state:visstate) =
    let state_without_movement = state |> set_sensorstate |> handle_input    
    let (abz_car,ai_cars) = (state_without_movement.abz_car,state_without_movement.ai_cars)
    let newabzcar = move_car abz_car
    let limit_elems (elems:'a list) = if elems.Length > 1000 then List.take 1000 elems else elems
    {|state_without_movement with 
        abz_car = move_car newabzcar;
        ai_cars = List.map (apply_autopilot >> move_car) ai_cars;
        speed_history = limit_elems (newabzcar.speed :: state.speed_history)
        distance_history = limit_elems (sensors.read_range_radar_sensor() :: state.distance_history)
        steering_history = limit_elems (sensors.get_steering_angle_degree() :: state.distance_history)
    |}

let init_visstate:visstate = {|
    abz_car = initcar;
    ai_cars = ai_cars;
    time = 0.0;
    brakepedal = false;
    gaspedal = false;
    speed_history = List.empty;
    distance_history = List.empty;
    steering_history = List.empty;
|}

drawloop init_visstate updatefn drawAll