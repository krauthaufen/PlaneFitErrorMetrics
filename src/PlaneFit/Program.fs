open PlaneFit

open System
open FSharp.Data.Adaptive

open Aardvark.Base
open Aardvark.Rendering
open Aardvark.Application
open Aardvark.Application.Slim
open Aardvark.Service
open Aardvark.UI
open Aardium
open Suave
open Suave.WebPart
open Aardvark.Geometry
open PocketPython

type RegressionInfo3d =
    {
        Plane           : Plane3d
        Trafo           : Trafo3d
        Sizes           : V3d
        Eigenvalues     : V3d
        AngularErrors   : V2d
    }

    member x.Center = x.Trafo.Forward.C3.XYZ
    member x.Normal = x.Trafo.Forward.C2.XYZ
    member x.Axis0 = x.Trafo.Forward.C0.XYZ
    member x.Axis1 = x.Trafo.Forward.C1.XYZ

type LinearRegression3d with
    member x.GetRegressionInfo(?confidenceLevel : float, ?degreesOfFreedom : int) =
        let degreesOfFreedom = defaultArg degreesOfFreedom 2
        let confidenceLevel = defaultArg confidenceLevel 0.95
        let struct(trafo, size) = x.GetTrafoAndSizes()
       
        let ev = size ** 2.0 
        let angles = 
            let inline fppf (x : float) (d1 : int) (d2 : int) =
                MathNet.Numerics.Distributions.FisherSnedecor.InvCDF(float d1, float d2, x)

            let inline fisherStatistic (n : int) (confidence : float) (dof : int) : float =
                fppf confidence dof (n - dof)
            
            let inline applyErrorScaling (nominal : V3d) (err : V3d) (n : int) : V3d =
                nominal * V3d.PPN - err |> abs

            let measurementNoise = ev.Z / (float (x.Count - degreesOfFreedom))
            let noiseCov = 4.0 * ev * measurementNoise

            let z = fisherStatistic x.Count confidenceLevel degreesOfFreedom
            let err = z * sqrt noiseCov

            let n = applyErrorScaling ev err x.Count |> sqrt

            V2d(
                atan2 n.Z n.X,
                atan2 n.Z n.Y
            ) 
        {
            Plane           = Plane3d(Vec.normalize trafo.Forward.C2.XYZ, trafo.Forward.C3.XYZ)
            Trafo           = trafo
            Sizes           = size
            Eigenvalues     = ev
            AngularErrors   = angles
        }

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
        Array.init 35 (fun _ ->
            let pos = rand.UniformV2d() * 10.0 - 5.0
            let err = (rand.UniformDouble() * 2.0 - 1.0) * 0.7
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
    Log.line "center: %s" (info.Center.ToString "0.00000")
    Log.line "eigen:  %s" (info.Eigenvalues.ToString "0.00000")
    Log.line "angle:  %s" ((Constant.DegreesPerRadian * info.AngularErrors).ToString "0.00000")
    Log.stop()
    
    exit 0

    Aardvark.Init()
    Aardium.init()

    let app = new OpenGlApplication()

    WebPart.startServerLocalhost 4321 [
        MutableApp.toWebPart' app.Runtime false (App.start App.app)
    ] |> ignore
    
    Aardium.run {
        title "Aardvark rocks \\o/"
        width 1024
        height 768
        url "http://localhost:4321/"
    }

    0