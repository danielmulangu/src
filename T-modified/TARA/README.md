# Trajectory Aware Rate Adaptation for the Network Simulator 3 (ns-3)

This project repository holds the source code developed for TARA, using the [network simulator 3 (ns-3)](https://www.nsnam.org). TARA uses the knowledge of flying nodes' movement to predict future channel conditions and perform rate adaptation accordingly.

Copyright (C) 2023, INESC TEC. This project is licensed under the terms of the *[GNU General Public Licence (GPL) v3.0](LICENSE)*

## Authors

This project is authored by:

* Ruben Queiros, Jose Ruela, Helder Fontes, Rui Campos [INESC TEC, Portugal]

## TARA Setup

The instructions to set-up and run this project are explained below.

### Setup ns-3 and Dependencies

1. Download the ns-3.38 version of [ns-3](https://gitlab.com/nsnam/ns-3-dev/-/tree/ns-3.38?ref_type=tags).

  ```shell
  git clone https://gitlab.com/nsnam/ns-3-dev.git
  cd ns-3-dev/ && git checkout tags/ns-3.38
  ```

2. Move TARA project to the scratch folder, which is inside the ns-3 project folder.

  ```shell
  mv tara/ ns-3-dev/scratch/
  ```

3. When inside TARA project, run the following commands to include TARA wifi manager in ns-3 source code.

  ```shell
  cp lib/link-adaptation/ns-3.38-CMakeLists.txt ../../src/wifi/CMakeLists.txt
  cp lib/link-adaptation/tara-wifi-manager.* ../../src/wifi/model/rate-control/
  ```

### Build ns-3

Back into the ns-3 root project folder, run the following commands to build ns-3:

```shell
./ns3 configure
./ns3 build
```

## Running TARA

To run TARA with the default configuration, run the following command:

```shell
./ns3 run "scratch/tara/sim" 
```

If you want to tweak the simulation, changing the **Rate Adaptation Algorithm** configured, as well as the **Simulation Random Seed**, consider the following Command Line Arguments: 

```shell
./ns3 run "scratch/tara/sim --simSeed=1 --raAlg=tara"
./ns3 run "scratch/tara/sim --simSeed=2 --raAlg=min"
./ns3 run "scratch/tara/sim --simSeed=3 --raAlg=id"
```

NOTE: The log files that result from the simulation, are saved in the *ns-3* root folder, under the names of `throughput.csv`, `distances.csv` and `positions.csv`.

## Cite this project.

If you would like to use this code, please cite it as follows:

* Queiros, R., Ruela, J., Fontes, H., & Campos, R. (2023). Trajectory-Aware Rate Adaptation for Flying Networks. https://doi.org/10.48550/arXiv.2307.06905

