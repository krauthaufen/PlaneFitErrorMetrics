namespace PocketPython

open System
open System.Net
open System.IO
open System.IO.Compression

open System
open System.Diagnostics
open System.Text
open System.Collections.Generic

module Pocket =

    type Result = { exitCode : int; stderr : string[]; stdout : string[] }

    let runProc (logger : string -> unit) filename args startDir env (stdinText : Option<string>) = 
        let timer = Stopwatch.StartNew()
        let procStartInfo = 
            ProcessStartInfo(
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                FileName = filename,
                Arguments = args,
                RedirectStandardInput = Option.isSome stdinText
            )
        match startDir with | Some d -> procStartInfo.WorkingDirectory <- d | _ -> ()
        let paths = procStartInfo.EnvironmentVariables.["Path"]
        let f = String.concat ";" (List.map Path.GetFullPath env)
        procStartInfo.EnvironmentVariables.["Path"] <- f + ";" + paths
        procStartInfo.UseShellExecute <- false

        let outputs = List<string>()
        let errors = List<string>()
        let outputHandler f (_sender:obj) (args:DataReceivedEventArgs) = logger args.Data; f args.Data
        let p = new Process(StartInfo = procStartInfo)
        p.OutputDataReceived.AddHandler(DataReceivedEventHandler (outputHandler outputs.Add))
        p.ErrorDataReceived.AddHandler(DataReceivedEventHandler (outputHandler errors.Add))
        let started = 
            try
                p.Start()
            with | ex ->
                ex.Data.Add("filename", filename)
                reraise()
        if not started then
            failwithf "Failed to start process %s" filename
        logger <| sprintf "Started %s with pid %i" p.ProcessName p.Id
        p.BeginOutputReadLine()
        p.BeginErrorReadLine()
        match stdinText with
            | None -> ()
            | Some t -> 
                p.StandardInput.Write(t)
                p.StandardInput.WriteLine("")
                p.StandardInput.Flush()
                p.StandardInput.Close()
        p.WaitForExit()
        timer.Stop()
        logger <| sprintf "Finished %s after %A milliseconds" filename timer.ElapsedMilliseconds
        let cleanOut l = l |> Seq.filter (fun o -> String.IsNullOrEmpty o |> not) |> Seq.toArray
        { exitCode = p.ExitCode; stderr = cleanOut errors; stdout = cleanOut outputs }

    let mutable url = "https://www.python.org/ftp/python/3.7.1/python-3.7.1-embed-amd64.zip"



    let runIn (logger : string -> unit) (workingDirectory : string) (packages : list<string>)  = 
        let target = Path.Combine(workingDirectory,"python")
        use wc = new WebClient()
        let scripts = [Path.Combine(target,"Scripts"); Path.Combine(target,"Lib")]
        if not (Directory.Exists target) then

            logger "[pocket-python] downloading python environment"
            let download = Path.Combine(workingDirectory, "python.zip")
            let d = wc.DownloadFile(url, download)
            Directory.CreateDirectory target |> ignore
            ZipFile.ExtractToDirectory(download, target)

            let pthPath = Path.Combine(target,"python37._pth")
            let pth = File.ReadAllText(pthPath)
            File.WriteAllText(pthPath, pth.Replace("#import site","import site"))

            wc.DownloadFile("https://bootstrap.pypa.io/get-pip.py", Path.Combine(target,"get-pip.py"))
            logger "[pocket-python] installing pip"
            runProc logger (Path.Combine(target,"python.exe")) "get-pip.py" (Some target) scripts None |> printfn "%A"

        let packages = String.concat " " packages
        let r = runProc logger (Path.Combine(target,"Scripts","pip.exe")) (sprintf "install %s" packages) (Some target) scripts None  
        if r.exitCode <> 0 then
            failwithf "%A" r.stderr
        else
            fun (script : string) (args : string) (eval : string) -> 
                let file = Path.ChangeExtension(Path.GetTempFileName(),"py")
                File.WriteAllText(file,script)
                let args = sprintf "%s %s" file args
                runProc logger (Path.Combine(target,"python.exe")) args (Some target) scripts (Some eval) 


    let run (packages : list<string>) (code : string) (args : string) (evalCode : string) = runIn ignore System.Environment.CurrentDirectory packages code args evalCode
    