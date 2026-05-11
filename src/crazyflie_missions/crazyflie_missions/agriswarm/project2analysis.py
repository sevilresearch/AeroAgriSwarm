import matplotlib.pyplot as plt
import numpy as np
from collections import Counter
import re

# Function to parse the data from the provided text or file
def extract_data(file_path):
    runs = []
    cells_travelled = {1: [], 2: [], 3: []}  # Dictionary to store data per agent
    revisits = {1: [], 2: [], 3: []}
    percent_revisits = {1: [], 2: [], 3: []}
    run_times = []  # List to store time taken per run

    with open(file_path, 'r') as file:
        content = file.read()

    # Regular expressions to match the data (case-insensitive for "run")
    run_pattern = re.compile(r"--- [Rr]un (\d+) ---")
    agent_pattern = re.compile(r"Agent (\d) \(\w+\) stats:")
    cells_pattern = re.compile(r"Total cells travelled: (\d+)")
    revisits_pattern = re.compile(r"Total revisits: (\d+)")
    percent_pattern = re.compile(r"Percentage of revisited cells: ([\d\.]+)%")
    time_pattern = re.compile(r"Time taken for Run \d+: ([\d\.]+) seconds")

    # Find all runs
    run_matches = list(re.finditer(run_pattern, content))
    run_indices = [match.start() for match in run_matches] + [len(content)]

    # Parse data run by run
    for i, run_match in enumerate(run_matches):
        run_num = int(run_match.group(1))
        runs.append(run_num)
        run_start = run_match.start()
        run_end = run_indices[i + 1] if i + 1 < len(run_indices) else len(content)
        run_content = content[run_start:run_end]

        # Find all agents in this run
        agent_matches = list(re.finditer(agent_pattern, run_content))
        for j, agent_match in enumerate(agent_matches):
            agent_num = int(agent_match.group(1))
            agent_start = agent_match.start()
            agent_end = agent_matches[j + 1].start() if j + 1 < len(agent_matches) else len(run_content)
            agent_content = run_content[agent_start:agent_end]

            # Extract data for this agent
            cells = re.search(cells_pattern, agent_content)
            rev = re.search(revisits_pattern, agent_content)
            perc = re.search(percent_pattern, agent_content)

            if cells and rev and perc:
                cells_travelled[agent_num].append(int(cells.group(1)))
                revisits[agent_num].append(int(rev.group(1)))
                percent_revisits[agent_num].append(float(perc.group(1)))

        # Extract time taken for this run
        time_match = re.search(time_pattern, run_content)
        if time_match:
            run_times.append(float(time_match.group(1)))

    # Flatten data for overall analysis
    all_cells_travelled = [val for agent_data in cells_travelled.values() for val in agent_data]
    all_revisits = [val for agent_data in revisits.values() for val in agent_data]
    all_percent_revisits = [val for agent_data in percent_revisits.values() for val in agent_data]

    return runs, cells_travelled, revisits, percent_revisits, run_times, all_cells_travelled, all_revisits, all_percent_revisits

# File path to your log file
# file_path = 'robot1analysisProject2.txt'
# file_path = 'robot2analysisProject2.txt'
# file_path = 'robot3analysisProject2.txt'
file_path = 'robot3analysisProject2_aftercorrections.txt'
# file_path = 'project2analysis_w_time.txt'

# Extract data
runs, cells_travelled, revisits, percent_revisits, run_times, all_cells_travelled, all_revisits, all_percent_revisits = extract_data(file_path)

# Print extracted data for verification
print(f"Runs: {runs}")
print(f"Cells travelled per agent: {cells_travelled}")
print(f"Revisits per agent: {revisits}")
print(f"Percentage of revisits per agent: {percent_revisits}")
print(f"Time taken per run (seconds): {run_times}")

# Calculate averages
average_travelled = np.mean(all_cells_travelled)
average_revisits = np.mean(all_revisits)
average_percent_revisits = np.mean(all_percent_revisits)
average_run_time = np.mean(run_times)

# Create a single figure with 4 subplots
fig, axs = plt.subplots(4, 1, figsize=(12, 24))  # Increased height for 4 subplots

# Colors for agents
agent_colors = {1: 'b', 2: 'y', 3: 'm'}  # Blue, Yellow, Magenta

# 1. Plotting Total Cells Travelled vs Simulation Runs
for agent, data in cells_travelled.items():
    if data:  # Plot only if there is data
        axs[0].plot(runs, data, marker='o', color=agent_colors[agent], label=f"Agent {agent}")
axs[0].axhline(y=average_travelled, color='r', linestyle='--', label=f"Avg: {average_travelled:.2f}")
axs[0].set_xlabel('Simulation Run')
axs[0].set_ylabel('Total Cells Travelled')
axs[0].set_title('Total Cells Travelled vs Simulation Runs')
axs[0].legend()
axs[0].grid(True)

# 2. Plotting Revisits vs Simulation Runs
for agent, data in revisits.items():
    if data:
        axs[1].plot(runs, data, marker='o', color=agent_colors[agent], label=f"Agent {agent}")
axs[1].axhline(y=average_revisits, color='r', linestyle='--', label=f"Avg: {average_revisits:.2f}")
axs[1].set_xlabel('Simulation Run')
axs[1].set_ylabel('Revisits')
axs[1].set_title('Revisits vs Simulation Runs')
axs[1].legend()
axs[1].grid(True)

# 3. Plotting Time Taken vs Simulation Runs
axs[2].plot(runs, run_times, marker='o', color='g', label="Time Taken")
axs[2].axhline(y=average_run_time, color='r', linestyle='--', label=f"Avg: {average_run_time:.2f} s")
axs[2].set_xlabel('Simulation Run')
axs[2].set_ylabel('Time Taken (seconds)')
axs[2].set_title('Time Taken vs Simulation Runs')
axs[2].legend()
axs[2].grid(True)

# 4. Bar Plot for Distribution of Revisits
frequency = Counter(all_revisits)
sorted_frequency = sorted(frequency.items())
x_values = [item[0] for item in sorted_frequency]
y_values = [item[1] for item in sorted_frequency]

axs[3].bar(x_values, y_values, color='purple', edgecolor='black')
axs[3].set_xlabel('Number of Revisits')
axs[3].set_ylabel('Frequency')
axs[3].set_title('Distribution of Revisits')
if x_values:  # Check if x_values is not empty
    axs[3].set_xticks(range(min(x_values), max(x_values) + 1, 1))
else:
    axs[3].set_xticks([])  # Set empty ticks if no data
axs[3].grid(True, axis='y')

# Increase spacing between subplots
plt.subplots_adjust(hspace=0.6)

# Show the figure
plt.show()

# Print Average Values
print(f"Average number of cells travelled: {average_travelled:.2f}")
print(f"Average number of revisits: {average_revisits:.2f}")
print(f"Average percentage of revisits: {average_percent_revisits:.2f}%")
print(f"Average time taken per run: {average_run_time:.2f} seconds")