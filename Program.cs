using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Diagnostics;
using System.IO;

namespace CPF_experiment
{
    /// <summary>
    /// This is the entry point of the application. 
    /// </summary>
    class Program
    {
        public static int numOfExecutions = 0;
        public static double replanTime = 0;
        public static int replanCounter = 0;
        private static string RESULTS_FILE_NAME = "Results.csv"; // Overridden by Main
        private static bool onlyReadInstances = false;
        public static bool TO_EXECUTE = true;

        /// <summary>
        /// Simplest run possible with a randomly generated problem instance.
        /// </summary>
        public void SimpleRun()
        {
            Run runner = new Run();
            runner.OpenResultsFile(RESULTS_FILE_NAME);
            runner.PrintResultsFileHeader();
            ProblemInstance instance = runner.GenerateProblemInstance(10, 3, 10);
            instance.Export("Test.instance");
            runner.SolveGivenProblem(instance, TO_EXECUTE);
            runner.CloseResultsFile();
        }

        /// <summary>
        /// Runs a single instance, imported from a given filename.
        /// </summary>
        /// <param name="fileName"></param>
        public void RunInstance(string fileName)
        {
            ProblemInstance instance;
            try
            {
                instance = ProblemInstance.Import(Directory.GetCurrentDirectory() + "\\Instances\\" + fileName);
            }
            catch (Exception e)
            {
                Console.WriteLine(String.Format("Skipping bad problem instance {0}. Error: {1}", fileName, e.Message));
                return;
            }

            Run runner = new Run();
            bool resultsFileExisted = File.Exists(RESULTS_FILE_NAME);
            runner.OpenResultsFile(RESULTS_FILE_NAME);
            if (resultsFileExisted == false)
                runner.PrintResultsFileHeader();
            runner.SolveGivenProblem(instance, TO_EXECUTE);
            runner.CloseResultsFile();
        }

        /// <summary>
        /// Runs a set of experiments.
        /// This function will generate a random instance (or load it from a file if it was already generated)
        /// </summary>
        public void RunExperimentSet(int[] gridSizes, int[] agentListSizes, int[] obstaclesProbs, int instances)
        {
            writeToFile(true, null, null, null, null, null, null, null, null, null);

            ProblemInstance instance;
            string instanceName;
            Run runner = new Run();

            bool resultsFileExisted = File.Exists(RESULTS_FILE_NAME);
            runner.OpenResultsFile(RESULTS_FILE_NAME);
            if (resultsFileExisted == false)
                runner.PrintResultsFileHeader();

            bool continueFromLastRun = false;
            string[] LastProblemDetails = null;
            string currentProblemFileName = Directory.GetCurrentDirectory() + "\\Instances\\current problem-" + Process.GetCurrentProcess().ProcessName;
            if (File.Exists(currentProblemFileName)) //if we're continuing running from last time
            {
                var lastProblemFile = new StreamReader(currentProblemFileName);
                LastProblemDetails = lastProblemFile.ReadLine().Split(',');  //get the last problem
                lastProblemFile.Close();
                continueFromLastRun = true;
            }

            for (int gs = 0; gs < gridSizes.Length; gs++)
            {
                for (int obs = 0; obs < obstaclesProbs.Length; obs++)
                {
                    runner.ResetOutOfTimeCounters();
                    for (int ag = 0; ag < agentListSizes.Length; ag++)
                    {
                        if (gridSizes[gs] * gridSizes[gs] * (1 - obstaclesProbs[obs] / 100) < agentListSizes[ag]) // Probably not enough room for all agents
                            continue;
                        for (int i = 0; i < instances; i++)
                        {
                            string allocation = Process.GetCurrentProcess().ProcessName.Substring(1);
                            //if (i % 33 != Convert.ToInt32(allocation)) // grids!
                            //    continue;

                            //if (i % 5 != 0) // grids!
                            //    continue;

                            if (continueFromLastRun)  //set the latest problem
                            {
                                gs = int.Parse(LastProblemDetails[0]);
                                obs = int.Parse(LastProblemDetails[1]);
                                ag = int.Parse(LastProblemDetails[2]);
                                i = int.Parse(LastProblemDetails[3]);
                                for (int j = 4; j < LastProblemDetails.Length; j++)
                                {
                                    runner.outOfTimeCounters[j - 4] = int.Parse(LastProblemDetails[j]);
                                }
                                continueFromLastRun = false;
                                continue; // "current problem" file describes last solved problem, no need to solve it again
                            }
                            if (runner.outOfTimeCounters.Length != 0 &&
                                runner.outOfTimeCounters.Sum() == runner.outOfTimeCounters.Length * Constants.MAX_FAIL_COUNT) // All algs should be skipped
                                break;
                            instanceName = "Instance-" + gridSizes[gs] + "-" + obstaclesProbs[obs] + "-" + agentListSizes[ag] + "-" + i;
                            try
                            {
                                instance = ProblemInstance.Import(Directory.GetCurrentDirectory() + "\\Instances\\" + instanceName);
                                instance.instanceId = i;
                            }
                            catch (Exception importException)
                            {
                                if (onlyReadInstances)
                                {
                                    Console.WriteLine("File " + instanceName + "  dosen't exist");
                                    return;
                                }

                                instance = runner.GenerateProblemInstance(gridSizes[gs], agentListSizes[ag], obstaclesProbs[obs] * gridSizes[gs] * gridSizes[gs] / 100);
                                instance.ComputeSingleAgentShortestPaths(); // REMOVE FOR GENERATOR
                                instance.instanceId = i;
                                instance.Export(instanceName);
                                instance.fileName = instanceName;
                            }

                            runner.SolveGivenProblem(instance, TO_EXECUTE);

                            //return; // DELETE ME!!!!!!!!!!!!!!!!!!!!!!!!!@# JUST FOR DEBUGGING!!!

                            // Save the latest problem
                            try
                            {
                                if (File.Exists(currentProblemFileName))
                                    File.Delete(currentProblemFileName);
                            }
                            catch
                            {
                                ;
                            }
                            var lastProblemFile = new StreamWriter(currentProblemFileName);
                            lastProblemFile.Write("{0},{1},{2},{3}", gs, obs, ag, i);
                            for (int j = 0; j < runner.outOfTimeCounters.Length; j++)
                            {
                                lastProblemFile.Write("," + runner.outOfTimeCounters[j]);
                            }
                            lastProblemFile.Close();
                        }
                    }
                }
            }
            runner.CloseResultsFile();
        }

        protected static readonly string[] daoMapFilenames = {/* "dao_maps\\den502d.map", "dao_maps\\ost003d.map", */"dao_maps\\brc202d.map" };

        protected static readonly string[] mazeMapFilenames = { "mazes-width1-maps\\maze512-1-6.map", "mazes-width1-maps\\maze512-1-2.map",
                                                "mazes-width1-maps\\maze512-1-9.map" };


        public static Stopwatch sw = new Stopwatch();
        /// <summary>
        /// Dragon Age experiment
        /// </summary>
        /// <param name="numInstances"></param>
        /// <param name="mapFileNames"></param>
        public void RunDragonAgeExperimentSet(int numInstances, string[] mapFileNames)
        {
            writeToFile(true, null, null, null, null, null, null, null, null, null);

            ProblemInstance instance;
            string instanceName;
            Run runner = new Run();

            bool resultsFileExisted = File.Exists(RESULTS_FILE_NAME);
            runner.OpenResultsFile(RESULTS_FILE_NAME);
            if (resultsFileExisted == false)
                runner.PrintResultsFileHeader();

            TextWriter output;
            int[] agentListSizes = {30};

            bool continueFromLastRun = false;
            string[] lineParts = null;

            string currentProblemFileName = Directory.GetCurrentDirectory() + "\\Instances\\current problem-" + Process.GetCurrentProcess().ProcessName;
            if (File.Exists(currentProblemFileName)) //if we're continuing running from last time
            {
                TextReader input = new StreamReader(currentProblemFileName);
                lineParts = input.ReadLine().Split(',');  //get the last problem
                input.Close();
                continueFromLastRun = true;
            }
            for (int ag = 0; ag < agentListSizes.Length; ag++)
            {
                for (int i = 0; i < numInstances; i++)
                {
                    string name = Process.GetCurrentProcess().ProcessName.Substring(1);


                    for (int map = 0; map < mapFileNames.Length; map++)
                    {
                        if (continueFromLastRun) // Set the latest problem
                        {
                            ag = int.Parse(lineParts[0]);
                            i = int.Parse(lineParts[1]);
                            map = int.Parse(lineParts[2]);
                            for (int j = 3; j < lineParts.Length && j - 3 < runner.outOfTimeCounters.Length; j++)
                            {
                                runner.outOfTimeCounters[j - 3] = int.Parse(lineParts[j]);
                            }
                            continueFromLastRun = false;
                            continue;
                        }
                        if (runner.outOfTimeCounters.Sum() == runner.outOfTimeCounters.Length * 20) // All algs should be skipped
                            break;
                        string mapFileName = mapFileNames[map];
                        instanceName = Path.GetFileNameWithoutExtension(mapFileName) + "-" + agentListSizes[ag] + "-" + i;
                        try
                        {
                            instance = ProblemInstance.Import(Directory.GetCurrentDirectory() + "\\Instances\\" + instanceName);
                        }
                        catch (Exception importException)
                        {
                            if (onlyReadInstances)
                            {
                                Console.WriteLine("File " + instanceName + "  dosen't exist");
                                return;
                            }

                            instance = runner.GenerateDragonAgeProblemInstance(mapFileName, agentListSizes[ag]);
                            instance.ComputeSingleAgentShortestPaths(); // Consider just importing the generated problem after exporting it to remove the duplication of this line from Import()
                            instance.instanceId = i;
                            instance.Export(instanceName);
                        }

                        runner.SolveGivenProblem(instance, TO_EXECUTE);

                        //save the latest problem
                        try
                        {
                            File.Delete(currentProblemFileName);
                        }
                        catch
                        {
                            ;
                        }
                        output = new StreamWriter(currentProblemFileName);
                        output.Write("{0},{1},{2}", ag, i, map);
                        for (int j = 0; j < runner.outOfTimeCounters.Length; j++)
                        {
                            output.Write("," + runner.outOfTimeCounters[j]);
                        }
                        output.Close();
                    }
                }
                
                runner.CloseResultsFile();
            }
        }

        public void writeToFile(bool header, string solver, string planTime, string elapsedTime, string planCost, string executionCost, string instanceId, string agentsCount, Run runner, string mapFileName)
        {
            string pathDesktop = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
            string filePath = pathDesktop + "\\RobustCSV.csv";

            string delimter = ",";
            List<string[]> output = new List<string[]>();
            string[] temps = new string[19];

            if (header && File.Exists(filePath))
                return;

            if (!File.Exists(filePath))
            {
                File.Create(filePath).Close();
            }
            if (header)
            {
                temps[0] = "IncstanceId";
                temps[1] = "#Agents";
                temps[2] = "Delay Prob.";
                temps[3] = "Solver";
                temps[4] = "Policy";
                temps[5] = "Robustness";
                temps[6] = "Obstacles";
                temps[7] = "Plan Cost";
                temps[8] = "Success";
                temps[9] = "Plan time";
                temps[10] = "No Collisions";
                temps[11] = "File Name";
            }
            output.Add(temps);

            int length = output.Count;
            using (System.IO.TextWriter writer = File.AppendText(filePath))
            {
                for (int index = 0; index < length; index++)
                {
                    writer.WriteLine(string.Join(delimter, output[index]));
                }
            }
        }




        /// <summary>
        /// This is the starting point of the program. 
        /// </summary>
        static void Main(string[] args)
        {
            Program me = new Program();
            Program.RESULTS_FILE_NAME = Process.GetCurrentProcess().ProcessName + ".csv";
            TextWriterTraceListener tr1 = new TextWriterTraceListener(System.Console.Out);
            Debug.Listeners.Add(tr1);


            if (Directory.Exists(Directory.GetCurrentDirectory() + "\\Instances") == false)
            {
                Directory.CreateDirectory(Directory.GetCurrentDirectory() + "\\Instances");
            }

            Program.onlyReadInstances = false;

            int instances = 50;

            bool runDragonAge = false;
            bool runGrids = true;
            bool runMazesWidth1 = false;
            bool runSpecific = false;

            numOfExecutions = 1;

            if (runGrids == true)
            {
                int[] gridSizes = new int[] {8};

                int[] agentListSizes = new int[] {8};
                // Note that success rate drops almost to zero for EPEA* and A*+OD/SIC on 40 agents.
            
                int[] obstaclesPercents = new int[] {0};
                me.RunExperimentSet(gridSizes, agentListSizes, obstaclesPercents, instances);
            }
            else if (runDragonAge == true)
                me.RunDragonAgeExperimentSet(instances, Program.daoMapFilenames); // Obstacle percents and grid sizes built-in to the maps.
            else if (runMazesWidth1 == true)
                me.RunDragonAgeExperimentSet(instances, Program.mazeMapFilenames); // Obstacle percents and grid sizes built-in to the maps.
            else if (runSpecific == true)
            { 
                me.RunInstance("test2");
            }
            Console.WriteLine("*********************THE END**************************");
            Console.ReadLine();
        }    
    }
}
