open System
open FSharp.Data.Adaptive
open Aardvark.Base
open Aardvark.Rendering
open Aardvark.Application
open Aardvark.Application.Utilities
open Aardvark.Geometry
open PocketPython
open Aardvark.SceneGraph

type RegressionInfo3d =
    {
        Plane           : Plane3d
        PlaneToWorld    : Trafo3d
        Eigenvalues     : V3d
        AngularErrors   : V2d
    }

    member x.Center = x.PlaneToWorld.Forward.C3.XYZ
    member x.Normal = x.PlaneToWorld.Forward.C2.XYZ
    member x.Axis0 = x.PlaneToWorld.Forward.C0.XYZ
    member x.Axis1 = x.PlaneToWorld.Forward.C1.XYZ
    member x.Ellipsoid = 
        let e = Euclidean3d.FromTrafo3d(x.PlaneToWorld, 1E-8)
        Ellipsoid3d(e, x.Eigenvalues)

module private Helpers =

    let gamma z = 
        let lanczosCoefficients = [76.18009172947146;-86.50532032941677;24.01409824083091;-1.231739572450155;0.1208650973866179e-2;-0.5395239384953e-5]
        let rec sumCoefficients acc i coefficients =
            match coefficients with
            | []   -> acc
            | h::t -> sumCoefficients (acc + (h/i)) (i+1.0) t
        let gamma = 5.0
        let x = z - 1.0
        Math.Pow(x + gamma + 0.5, x + 0.5) * Math.Exp( -(x + gamma + 0.5) ) * Math.Sqrt( 2.0 * Math.PI ) * sumCoefficients 1.000000000190015 (x + 1.0) lanczosCoefficients

    let beta x y =
        gamma x * gamma y / gamma (x + y)     

    let invFCDF (x : float) (d1 : float) (d2 : float) : float =
        failwith "implement me"

type LinearRegression3d with
    member x.GetRegressionInfo(?confidenceLevel : float, ?degreesOfFreedom : int) =
        let degreesOfFreedom = defaultArg degreesOfFreedom 2
        let confidenceLevel = defaultArg confidenceLevel 0.95
        //let struct(trafo, size) = x.GetTrafoAndSizes()
       
        let c = x.Centroid
        let (u, s, _vt) = SVD.Decompose x.CovarianceMatrix |> Option.get

        let trafo =
            Trafo3d(
                M44d(
                    u.M00, u.M01, u.M02, c.X,
                    u.M10, u.M11, u.M12, c.Y,
                    u.M20, u.M21, u.M22, c.Z,
                    0.0, 0.0, 0.0, 1.0
                ),
                M44d(
                    u.M00, u.M10, u.M20, -Vec.dot u.C0 c,
                    u.M01, u.M11, u.M21, -Vec.dot u.C1 c,
                    u.M02, u.M12, u.M22, -Vec.dot u.C2 c,
                    0.0, 0.0, 0.0, 1.0
                )
            )

        let ev = s.Diagonal
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
            PlaneToWorld    = trafo
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
        Array.init 250 (fun _ ->
            let pos = rand.UniformV2d() * V2d(10.0, 6.0) - V2d(5.0, 3.0)
            let err = (rand.UniformDouble() * 2.0 - 1.0) * 0.9
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
    Log.line "normal: %s" (info.Normal.ToString "0.00000")
    // Log.line "axis0:  %s" (info.Axis0.ToString "0.00000")
    // Log.line "axis1:  %s" (info.Axis1.ToString "0.00000")
    Log.line "axis0:  %s" (info.Axis0.ToString "0.00000")
    Log.line "axis1:  %s" (info.Axis1.ToString "0.00000")
    Log.line "angle:  %s" ((Constant.DegreesPerRadian * info.AngularErrors).ToString "0.00000")
    Log.stop()
    
    Aardvark.Init()

    // use app = new OpenGlApplication()
    // use win = app.CreateGameWindow(8)

    let sg =
        Sg.ofList [
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