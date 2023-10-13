# Vector Field Based Volume Peeling for Multi-Axis Machining

## Abstract
This paper presents an easy-to-control volume peeling method for multi-axis machining based on the computation taken on vector fields. The current scalar field based methods are not flexible and the vector-field based methods do not guarantee the satisfaction of the constraints in the final results. We first conduct an optimization formulation to compute an initial vector field that is well aligned with those anchor vectors specified by users according to different manufacturing requirements. The vector field is further optimized to be an irrotational field so that it can be completely realized by a scalar field's gradients. Iso-surfaces of the scalar field will be employed as the layers of working surfaces for multi-axis volume peeling in the rough machining. Algorithms are also developed to remove and process singularities of the fields. Our method has been tested on a variety of models and verified by physical experimental machining. 


## Installation
Please compile the code with QMake file “ShapeLab.pro”.

Platform: Windows + Visual Studio + QT-plugin (tested version: VS2019 + QT5.12.3 + msvc2017_64)

Installation Steps:
- Install Visual Studio Extension plug-in (QT VS Tool) to open the *.pro file and generate the project
- Set 'ShapeLab' as the start up project
- Enable OpenMP to get best performace at: ShapeLab Property Pages -> Configuration Properties -> C/C++ -> Language -> Open MP Support -> Select 'Yes (/openmp)'
- Open Console at: ShapeLab Property Pages -> Configuration Properties -> Linker -> System -> SubSystem -> Select 'Console (/SUBSYSTEM:CONSOLE)'
- Support large obj at: ShapeLab Property Pages -> Configuration Properties -> C/C++ -> Command Line -> Additional Options -> add "/bigobj"
- Install Intel oneAPI Math Kernel Library (oneMKL download) and enable it at: ShapeLab Property Pages -> Configuration Properties -> Intel Libraries for oneAPI -> Intel oneAPI Math Kernel Library (oneMKL) -> Use oneMKL -> Select 'Parallel'
- And change the code generation method at: ShapeLab & QMeshLab & GLKLib Property Pages -> Configuration Properties -> C/C++ -> Code Generation -> Runtime Library -> Select 'Multi-threaded(/MT) for release configuration'. Note that this option will be 'Multi-threaded Debug (/MTd) for debug configuration.



## Usage
