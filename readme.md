# 1. Controller Parameter Tuning Script

This repository contains a MATLAB-based script for automating the tuning of controller parameters. The script utilizes symbolic computation to determine the parameters that satisfy the conditions established in the theoretical proofs.

## Overview

The script follows a systematic procedure for selecting controller parameters, including system functions, gains, and constants, ensuring that the controller meets the required stability and performance conditions.

## Requirements

- MATLAB (tested with version R2023b)
- Symbolic Math Toolbox

## Script Breakdown

The script executes the following steps:

a. The script verifies the necessary assumptions regarding system dynamics and other constants.

b. It then designs the controller by selecting appropriate parameters to ensure that the system behaves as desired.

c. The script assigns necessary gain parameters to satisfy the system's small-gain conditions for stability.

d. Based on the parameters selected, it recursively designs the controller for each subsystem to meet tracking and stability requirements.

## Usage

### Edit User-Defined Items
The script requires you to define certain parameters. These are marked with `[User-Defined]` comments in the code. Review and update these values based on your system specifications.

### Run the Script
Once the necessary parameters are defined, simply run the script in MATLAB. It will automatically calculate the required controller gains and display the results.

# 2. Positive Basis Generation

By running the main file in the folder `generate_positive_basis`, you can generate the desired positive basis vectors. These vectors are an essential part of the controller design process and are used to ensure that the system meets specific theoretical properties.

## Usage
- Navigate to the `generate_positive_basis` folder.
- Run the main file to generate the positive basis.
- The generated basis will be used by the controller.
