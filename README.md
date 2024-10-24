# FAST-OAD plugin for Regional Transport Aircraft (RTA)

## Introduction

This repository furnishes a FAST-OAD plugin for the design and analysis of Regional Transport Aircraft.
The models present in this repository rely on FAST-OAD for execution, see [FAST-OAD documentation](https://fast-oad.readthedocs.io/en/stable/documentation/custom_modules/index.html).

The models are based on the [PhD thesis](https://theses.fr/2023ESAE0020) of Vincenzo Palladino. Documentation can be found in his PhD thesis.

This README file explains how to install this repository.

It is assumed that you know how to use [Git](https://git-scm.com), 
[Python](https://www.python.org) and [FAST-OAD](https://github.com/fast-aircraft-design/FAST-OAD).

This repository is designed to use [Poetry](https://python-poetry.org) (version 1.4.2 or above)
for managing the development environment.
Instructions below assume you have it already installed. You may adapt them if you don't 
want to use Poetry.

_**Note**: In this document, any leading slash (`/`) in a path refers to project root._

## Setting up your project

After copying the content of this repository and initiating your own Git 
repository, you should take the following steps:

### Install your working environment
- Typing `poetry install` in your terminal will create a virtual environment and
  install all defined dependencies in it.

### Setup Git hooks
Simply run:
```bash
pre-commit install
```
(see below for more details)

### Pytest
[Pytest](https://docs.pytest.org/) is recommended for writing tests. The development environment
is set with code coverage tools.

**Pytest and its companions are configured in `/pyproject.toml` file.**

It is recommended to have unit tests in `tests` folders next to tested code.
Other kind of tests (integration, non-regression) should be in the `/tests` folder

Unit tests will be launched with simply:
```bash
pytest
```

Other tests will be launched with
```bash
pytest tests
```

### Tutorials
Tutorials are available in `/rta/notebooks`. You can launch them using jupyter lab:
```bash
jupyter lab
```

### Ruff
[![Ruff](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json)](https://github.com/astral-sh/ruff)
checks/corrects code style and automates the code formatting.

Kudos to [Black](https://black.readthedocs.io/en/stable) and 
[Flake8](https://flake8.pycqa.org/) that are very good tools. Yet, for a fresh start,
Ruff seems the way to go, since it does the same job as these two, only much faster.

**Ruff is configured in `/pyproject.toml` file.**

Coupled with pre-commit (see below) and/or integrated with your IDE, it
automates all the code formatting, and it is sooo good.

_**Note to PyCharm users**: there is a [ruff plugin](https://plugins.jetbrains.
com/plugin/20574-ruff)._

