cd %SCENARIO_RUNNER_ROOT%
start python run_hbs_experiment.py --title "HBS AV Experiment 1 - Pedestrian Crossing in Clear Weather" --route %SCENARIO_RUNNER_ROOT%\srunner\data\hbs_routes.xml  %SCENARIO_RUNNER_ROOT%\srunner\data\hbs_scenarios.json 4 --entities %SCENARIO_RUNNER_ROOT%\srunner\data\hbs_entities.xml --noBackgroundActivity --scenarioNames Scenario12

SLEEP 50

CD %CARLA_ROOT%\PythonAPI\examples
start python DReyeVR_AI.py -n 0

cd %SCENARIO_RUNNER_ROOT%
