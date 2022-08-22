cd %SCENARIO_RUNNER_ROOT%
start python run_hbs_experiment.py --title "HBS Social Experiment 1" --route %SCENARIO_RUNNER_ROOT%\srunner\data\hbs_routes.xml  %SCENARIO_RUNNER_ROOT%\srunner\data\hbs_scenarios.json 4 --debug --noBackgroundActivity --scenarioName Scenario18

CD %CARLA_ROOT%\PythonAPI\examples
start python DReyeVR_AI.py -n 0

cd %SCENARIO_RUNNER_ROOT%
