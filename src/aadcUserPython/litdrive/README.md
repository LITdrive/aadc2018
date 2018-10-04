# LITdrive Python

Towards Autonomy.

## Testing

Be sure to activate the environment if you are using Anaconda:

    conda activate aadc

You need to execute every module from the project root (here), and call the package like so:

    python -m litdrive.demo.sensors

## Requirements

Recreate the Python 3.7 environment with Anaconda:

	conda env create -f environment.yml
	
Or, alternativley, with pip:

    pip3 install -r requirements.txt

### Build Cython dependencies

The module `litdrive.util.darkflow` has Cython code that needs to be built first:

	python setup.py build_ext --inplace
