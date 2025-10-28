# All combinations of managing system environment variables (true/false for each)
managing_system_combinations=(
    "false false false"  # CONSIDER_DEPENDENCIES CONSIDER_CRITICALITY_LEVEL CONSIDER_COST_FUNCTION
    "false false true"
    "false true false"
    "false true true"
    "true false false"
    "true false true"
    "true true false"
    "true true true"
)

echo "Starting experiment with all managing system combinations..."

for combination in "${managing_system_combinations[@]}"; do
    # Parse the combination into individual variables
    read -r deps crit cost <<< "$combination"
    
    echo "Running with CONSIDER_DEPENDENCIES=$deps, CONSIDER_CRITICALITY_LEVEL=$crit, CONSIDER_COST_FUNCTION=$cost"
    
    # Export the environment variables and run the experiment
    CONSIDER_DEPENDENCIES=$deps CONSIDER_CRITICALITY_LEVEL=$crit CONSIDER_COST_FUNCTION=$cost \
    setsid ./evaluation/run_single_experiment.sh
    
    echo "experiment with combination ($deps,$crit,$cost) of set $i finished. Preparing for next experiment..."
    sleep 5
done

echo "All experiments completed."