cd %SCENARIO_RUNNER_ROOT%
start python run_hbs_experiment.py --title "Follow Leading Vehicle with Obstruction - Clear Weather " --route %SCENARIO_RUNNER_ROOT%\srunner\data\hbs_routes.xml  %SCENARIO_RUNNER_ROOT%\srunner\data\hbs_scenarios.json 3 --debug --noBackgroundActivity --scenarioNames Scenario15

SLEEP 50

CD %CARLA_BIN%\PythonAPI\examples
start python DReyeVR_AI.py -n 0 -s 1920 -d 5

cd %SCENARIO_RUNNER_ROOT%
