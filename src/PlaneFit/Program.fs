open System
open FSharp.Data.Adaptive
open Aardvark.Base
open Aardvark.Rendering
open Aardvark.Application
open Aardvark.Application.Utilities
open Aardvark.Geometry
open PocketPython
open Aardvark.SceneGraph
open Aardvark.Rendering.Text
open Chiron

module Bla = 

    let fTable() =
        let ds = [|1;2;3;4;5;6;7;8;9;10;15;20;25;30;35;40;45;50;55;60;70;80;90;100;200;400;500;600;700;800;900;1000;10000|]
        let ps = [0.8 .. 0.005 .. 0.996 ]

        let builder = System.Text.StringBuilder()
        let printfn fmt = Printf.kprintf (fun str -> builder.AppendLine str |> ignore) fmt
        let printf fmt = Printf.kprintf (fun str -> builder.Append str |> ignore) fmt
        printfn "module FDistr = "
        printfn "    let private ds = [| %s |]" (ds |> Seq.map (sprintf "%d") |> String.concat "; ")
        printfn "    let private ps = [| %s |]" (ps |> Seq.map (sprintf "%.3f") |> String.concat "; ")
        printfn "    let private invCDFTable = "
        printfn "        [|" 
        for p in ps do

            printfn "            // %f" p
            for i1, d1 in Array.indexed ds do
                printf "            "
                for i2, d2 in Array.indexed ds do
                    if i2 > 0 then printf "; "
                    try printf "%.15g" (MathNet.Numerics.Distributions.FisherSnedecor.InvCDF(float d1, float d2, p))
                    with _ -> printf "System.Double.NaN"
                printfn ""
            printfn ""
            //     printfn ""
            // printfn "        |]"
        printfn "        |]"
        
        let t = builder.ToString()
        System.IO.File.WriteAllText(@"bla.fs", t)

    let testFDistr() =
      
        let rand = Random()
        let mutable sum = 0.0
        let mutable maxErr = 0.0
        let mutable minErr = 1.0
        let iter = 100000
        for i in 1 .. iter do   
            let a = 2 + rand.Next(100) |> float
            let b = 2 + rand.Next(100) |> float
            let p = 
                let v = (float (rand.Next(9)) / 10.0)
                0.95 + v * 0.05
            let mine = FDistr.invCDF a b p

            printf "invCDF(%f, %f, %f): " a b p
            let test = MathNet.Numerics.Distributions.FisherSnedecor.InvCDF(a, b, p)
            let err = abs (mine - test) / test
            printfn "%.2f%%" (100.0 * err)
            sum <- sum + err
            maxErr <- max maxErr err
            minErr <- min minErr err
            if err >= 0.09 then
                failwithf "bad: %A %A %A: %A vs %A" a b p mine test
        printfn "min: %.2f%%" (100.0 * minErr)
        printfn "max: %.2f%%" (100.0 * maxErr)
        printfn "avg: %.2f%%" (100.0 * sum / float iter)

[<EntryPoint>]
let main args =
    
    let mutable reg = LinearRegression3d.empty

    let rand = RandomSystem()

    let frame =
        let z = rand.UniformV3dDirection()
        let y = Vec.cross z (rand.UniformV3dDirection()) |> Vec.normalize
        let x = Vec.cross y z
        let pos = rand.UniformV3d() * 10.0 - V3d(5.0)
        Log.start "input"
        Log.line "p: %s" (pos.ToString "0.000")
        Log.line "x: %s" (x.ToString "0.000")
        Log.line "y: %s" (y.ToString "0.000")
        Log.line "z: %s" (z.ToString "0.000")
        Log.stop()
        Trafo3d.FromBasis(x,y,z,pos)

    let points = 
        let bounds = Box2d.FromCenterAndSize(V2d.Zero, V2d(10.0, 6.0))
        let scale = V2d(10.0, 5.0)
        let maxError = 0.3
        Array.init 1024 (fun _ ->
            let pos = rand.UniformV2dDirection() * scale * rand.UniformDouble() //rand.UniformV2d(bounds)
            let err = (rand.UniformDouble() * 2.0 - 1.0) * maxError
            frame.Forward.TransformPos(V3d(pos, err))
        )

    let arr = 
        points |> Array.map (fun v -> sprintf "[%f, %f, %f]" v.X v.Y v.Z) |> String.concat "," 

    let code =
        String.concat "\r\n" [
            sprintf "from attitude import Orientation"
            sprintf "import numpy as np"
            sprintf "measurement = Orientation(np.array([%s]));" arr
            "print(\"center: \", measurement.center);"
            "print(\"eigen:  \", measurement.eigenvalues);"
            "print(\"normal: \", measurement.coefficients);"
            "print(\"axis0:  \", measurement.axes[0]);"
            "print(\"axis1:  \", measurement.axes[1]);"
            "print(\"hyp:    \", measurement.hyperbolic_axes);"
            "print(\"angle:  \", measurement.angular_errors());"
            
        ]

    Log.start "python Attitude"
    let res = Pocket.run ["numpy"; "scipy"; "colour"; "mplstereonet"; "matplotlib"; "jinja2"; "Attitude"] code "" ""
    for l in res.stdout do
        Log.line "%s" l
    for l in res.stderr do
        Log.warn "%s" l
    Log.stop()

    for p in points do
        reg <- reg.Add p

    Log.start "ours"
    let info = reg.GetRegressionInfo()
    Log.line "center: %s" (info.Center.ToString "0.00000000")
    Log.line "eigen:  %s" (info.Eigenvalues.ToString "0.00000000")
    Log.line "normal: %s" (info.Normal.ToString "0.00000000")
    Log.line "axis0:  %s" (info.Axis0.ToString "0.00000000")
    Log.line "axis1:  %s" (info.Axis1.ToString "0.00000000")
    Log.line "hyp:    %s" (info.HyperbolicAxes.ToString "0.00000000")
    Log.line "angle:  %s" ((Constant.DegreesPerRadian * info.AngularErrors).ToString "0.00000000")
    Log.stop()

    let json = info.ToJson(name = "hans", uid = "myuid", points = Seq.truncate 5 points) |> Json.formatWith JsonFormattingOptions.Pretty
    Log.line "%s" json
    exit 0
    Aardvark.Init()

    let ellipseXY =
        let scale = 1.0
        ShapeList.ofList [
            ConcreteShape.ellipse C4b.White 0.2 (Ellipse2d(V2d.Zero, scale * V2d.IO * sqrt info.Eigenvalues.X, scale * V2d.OI * sqrt info.Eigenvalues.Y))
        ]
     

    // use app = new OpenGlApplication()
    // use win = app.CreateGameWindow(8)

    let sg =
        Sg.ofList [
            // Sg.shape (AVal.constant ellipseXY)
            // |> Sg.transform info.PlaneToWorld
            
            Sg.draw IndexedGeometryMode.PointList
            |> Sg.vertexAttribute' DefaultSemantic.Positions (points |> Array.map V3f)
            |> Sg.uniform' "PointSize" 5.0
            |> Sg.shader {
                do! DefaultSurfaces.trafo
                do! DefaultSurfaces.pointSprite
                do! DefaultSurfaces.pointSpriteFragment
                do! DefaultSurfaces.constantColor C4f.Red
            }

            Sg.unitSphere' 4 C4b.White
            |> Sg.transform (Trafo3d.Scale(sqrt info.Eigenvalues) * info.PlaneToWorld)
            |> Sg.fillMode' FillMode.Line
            |> Sg.shader {
                do! DefaultSurfaces.trafo
                do! DefaultSurfaces.constantColor C4f.White
            }
        ]

    show {
        backend Backend.GL
        debug false
        scene sg
    }




    0