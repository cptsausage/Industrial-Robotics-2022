%% Script file for quick running of class functions

%% Quick ploting of site area and models
clf;
clear,

ConstructionSite = Assembly;

%% Validating the UR3 model using known qjoint file

ConstructionSite.ur3.ValidateModel;

%% MUST PERFORM SAFETY PLOT TO DETERMINE RANDOM BRICK POSITIONS
ConstructionSite.SafetyPlot();

%% Random brick locations
TotalBricks = 9;

ConstructionSite.RandomBricks(TotalBricks);

%% Specific brick locations

BrickPoses = [
    0 0 0; 
    0 0 0; 
    0 0 0; 
    0 0 0; 
    0 0 0; 
    0 0 0; 
    0 0 0; 
    0 0 0; 
    0 0 0];

ConstructionSite.SpecificBricks(BrickPoses);

%% Adjust brick location
ConstructionSite.bricks.brick{1}.base(2,4) = ConstructionSite.bricks.brick{1}.base(2,4) - 1;
animate(ConstructionSite.bricks.brick{1},0);

%% Perform wall assembly

ConstructionSite.Construction
