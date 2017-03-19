
let prefix : string ref = ref ""
let outFile : string ref = ref ""
let globalOutFile : string ref = ref ""
let merge : bool ref = ref false

let options = [
  
  ("--task",
   Arg.Set_string prefix,
   "Task name for global name prefix");

  
  "", Arg.Unit (fun () -> ()), "General:";
  "--out", Arg.Set_string outFile, "Set the name of the output file";
  "--merge", Arg.Set merge,
    "Operate in CIL merger mode";
]

let parseOneFile (fname: string) : Cil.file =
  let cabs, cil = Frontc.parse_with_cabs fname () in
  Rmtmps.removeUnusedTemps cil;
  cil

let outputFile (f : Cil.file) : unit =
  if !outFile <> "" then
    try
      let c = open_out !outFile in
      
      Cil.print_CIL_Input := false;
      Stats.time "printCIL" 
        (Cil.dumpFile (!Cil.printerForMaincil) c !outFile) f;
      close_out c
    with _ ->
      Errormsg.s (Errormsg.error "Couldn't open file %s" !outFile)

let processOneFile (cil: Cil.file) : unit =
(*  Subtask.addPrefix cil !prefix;
  Initializer.initial cil !prefix;
  Analyzeglobals.outputGlobalVars cil (!prefix ^ "__globvars.h");
  InductionBasic.inductionBasic cil !prefix;
*)  
  Inliner.doit cil;
  (*Inliner.feature;*)
  outputFile cil;
;;

let main () =
  Cil.print_CIL_Input := true;
  Cil.insertImplicitCasts := false;
  Cil.lineLength := 999999;
  Cil.warnTruncate := false;
  Errormsg.colorFlag := true;

  Cabs2cil.doCollapseCallCast := true;

  let usageMsg = "Usage: taskprefix prefix source-file" in
  Arg.parse options Ciloptions.recordFile usageMsg;
  Ciloptions.fileNames := List.rev !Ciloptions.fileNames;
  let files = List.map parseOneFile !Ciloptions.fileNames in
  let one =
    match files with
	  | [] -> Errormsg.s (Errormsg.error "No file names provided")
    | [o] -> o
    | _ -> Mergecil.merge files "stdout"
  in

  processOneFile one
;;

begin 
  try 
    main () 
  with
  | Frontc.CabsOnly -> ()
  | Errormsg.Error -> ()
end;
exit (if !Errormsg.hadErrors then 1 else 0)
