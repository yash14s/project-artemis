#!/bin/bash

#Shell script to create the required python environments
cd .. && cd environments
conda env create -f paenv.yml
conda env create -f ardupilotenv.yml