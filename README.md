# Jaguar
Jaguar is a <b>line following</b> and <b>maze traversing</b> bot made for SAC - SRA AutoSim competiton.


## Table of Contents

- [Project](#MazeBlaze-v2)
  - [Table of Contents](#table-of-contents)
  - [About The Project](#about-the-project)
  - [File Structure](#file-structure)
  - [Getting started](#Getting-Started)
  - [Contributors](#contributors)
  - [Acknowledgements and Resources](#acknowledgements-and-references)
  - [License](#license)
  
## About

Jaguar is a <b>line following</b> and <b>maze traversing</b> bot made for SAC - SRA AutoSim competiton.

  ### Domains Explored
Embedeed C, Control system, electronics, Basics of PCB design etc.

## File Structure
```
👨‍💻Jaguar
 ┣ 📂firmware                          // Code files 
   ┣ 📂1_lsa                           // Contains code for getting readings from LSA 
   ┃ ┣ 📂main                         
   ┃ ┃ ┗ 📄main.c 
   ┃ ┃ ┣ 📄CMakeList.txt
   ┃ ┣ 📄CMakeList.txt
   ┣ 📂2_line_following                // Contains code for line following
   ┣ 📂3_blind                         // Contains code for line following
   ┣ 📂4_obstacle_detection                // Contains code for line following
   ┣ 📂5_left_follow                   // Contains code for left-follow-rule and path planning
   ┣ 📂Components                      // Contains all the header and source files used in project
     ┣ 📂include                       
     ┣ 📂src 
     ┣ 📄CMakeList.txt
     
``` 
## Getting Started

### Prerequisites
To download and use this code, the minimum requirements are:

* [ESP_IDF](https://github.com/espressif/esp-idf)
* Windows 7 or later (64-bit), Ubuntu 20.04 or later
* [Microsoft VS Code](https://code.visualstudio.com/download) or any other IDE 

### Installation

Clone the project by typing the following command in your Terminal/CommandPrompt

```
git clone https://github.com/PritK99/Jaguar.git 
```
Navigate to the MazeBlaze-v2.1 folder

```
cd Jaguar
```

### Usage

Once the requirements are satisfied, you can easily download the project and use it on your machine.
After following the above steps , use the following commands to:

To activate the IDF

```
get_idf
```

To build the code 

```
idf.py build
```

To flash the code

```
idf.py -p (PORT) flash monitor
```

## Contributors

* [Prit Kanadiya](https://github.com/PritK99)
* [Sameer Gupta](https://github.com/PritK99)
* [Aryan Karawale](https://github.com/PritK99)
* [Rajat Kaushik](https://github.com/PritK99)

## Acknowledgements and References
* [SRA VJTI](https://sravjti.in/) 
* [SAC - SRA Autosim Competition](https://www.youtube.com/watch?v=VHxqYZSrtgY) video
* [Wall - E](https://github.com/SRA-VJTI/Wall-E.git) github repository
 
## License
[MIT License](https://opensource.org/licenses/MIT)



