Rhoban Model Learning
=====================

Taxonomy 
--------
- Sample: A couple composed of an Input and an Observation
- DataSet: Two sets of samples, one for training and one for validation
- DataSetReader: An entity allowing to read the data inside a file to extract a DataSet
- Model: Parametrized entities, model learning is finding the 'best' set of parameters
- Predictors: for a given model and a given input
- Prior: A distribution of probability on the parameters of a model

Targeted Models
---------------
- Mocap
- PosesOptimization
- PosesInference

Untested learning model
-----------------------
- Calibration Model (compiles but not tested)

Disabled learnings (due to new model)
-------------------------------------
