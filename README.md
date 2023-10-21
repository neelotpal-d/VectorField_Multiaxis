<!-- Google tag (gtag.js) -->
<script async src="https://www.googletagmanager.com/gtag/js?id=G-07BGR3LPXZ"></script>
<script>
  window.dataLayer = window.dataLayer || [];
  function gtag(){dataLayer.push(arguments)};
  gtag('js', new Date());
  gtag('config', 'G-07BGR3LPXZ');

</script>

(This is an extended version of the paper IDETC2023-116742, which got the [**Best Paper Award**](https://www.linkedin.com/posts/charlie-c-l-wang-8396189a_cadcam-cncmachining-digitalmanufacturing-activity-7099853958055084033-4qXs) at the ASME IDETC/CIE 2023 Conference, Boston, USA, August 20-23, 2023.)

![Pipeline of the method](./Images/pipeline.jpg)

## Abstract
This paper presents an easy-to-control volume peeling method for multi-axis machining based on the computation taken on vector fields. The current scalar field based methods are not flexible and the vector-field based methods do not guarantee the satisfaction of the constraints in the final results. We first conduct an optimization formulation to compute an initial vector field that is well aligned with those anchor vectors specified by users according to different manufacturing requirements. The vector field is further optimized to be an irrotational field so that it can be completely realized by a scalar field's gradients. Iso-surfaces of the scalar field will be employed as the layers of working surfaces for multi-axis volume peeling in the rough machining. Algorithms are also developed to remove and process singularities of the fields. Our method has been tested on a variety of models and verified by physical experimental machining. 

Read the paper: [Arxiv Pre-print](https://arxiv.org/pdf/2308.00472.pdf)   &emsp;  |  &emsp;   [ASME JCISE](https://doi.org/10.1115/1.4063861)


## Video Summary
<iframe  width="100%" height= "480" src="https://www.youtube.com/embed/Bzt2oe6YYh8" title="Vector Field Based Volume Peeling for Multi-Axis Machining" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen> </iframe>




## Installation
Please compile the code with QMake file “ShapeLab.pro”.

Platform: Windows + Visual Studio + QT-plugin (tested version: VS2019 + QT5.12.3 + msvc2017_64)

Installation Steps:
- Install Visual Studio Extension plug-in (QT VS Tool) to open the *.pro file and generate the project
- Set 'ShapeLab' as the start up project
- Enable OpenMP to get best performace at: ShapeLab Property Pages -> Configuration Properties -> C/C++ -> Language -> Open MP Support -> Select 'Yes (/openmp)'
- Open Console at: ShapeLab Property Pages -> Configuration Properties -> Linker -> System -> SubSystem -> Select 'Console (/SUBSYSTEM:CONSOLE)'
- Install Intel oneAPI Math Kernel Library (oneMKL download) and enable it at: ShapeLab Property Pages -> Configuration Properties -> Intel Libraries for oneAPI -> Intel oneAPI Math Kernel Library (oneMKL) -> Use oneMKL -> Select 'Parallel'
- And change the code generation method at: ShapeLab & QMeshLab & GLKLib Property Pages -> Configuration Properties -> C/C++ -> Code Generation -> Runtime Library -> Select 'Multi-threaded(/MT) for release configuration'. Note that this option will be 'Multi-threaded Debug (/MTd) for debug configuration.
  
**_Go to [GitHub](https://github.com/neelotpal-d/VectorField_Multiaxis) to download the code and for more instructions_**



## Contact Information
Neelotpal Dutta ([neelotpal.dutta@manchester.ac.uk](mailto:neelotpal.dutta@manchester.ac.uk))\
Charlie C.L. Wang ([changling.wang@manchester.ac.uk](mailto:changling.wang@manchester.ac.uk))

