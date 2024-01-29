/**
  @page mdkarm MDK-ARM Project Template for STM32L1xx Ultra Low Power devices
  
  @verbatim
  ******************** (C) COPYRIGHT 2014 STMicroelectronics *******************
  * @file    readme.txt
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    16-May-2014
  * @brief   This sub-directory contains all the user-modifiable files needed 
  *          to create a new project linked with the STM32L1xx Standard Peripheral
  *          Library and working with RealView Microcontroller Development Kit(MDK-ARM)
  *          software toolchain.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2014 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
   @endverbatim
 
 @par Directory contents
 
 - Project.uvproj/.uvopt: A pre-configured project file with the provided library structure
                          that produces an executable image with MDK-ARM.

Enabling "Options for Target 'Output' Browser Information" is useful for quick 
source files navigation but may slow the compilation time.
 
 
 @par How to use it ?
 
 - Open the Project.Uv2 project
 - In the build toolbar select the project config:
     - STM32L1XX_MD(STM32L1xxxBxx): to configure the project for STM32L Ultra Low Power Medium-density devices.
                                    You can use STMicroelectronics STM32L152-EVAL board to run this project. 

     - STM32L1XX_MDP(STM32L1xxxCxx): to configure the project for STM32L Ultra Low Power Medium-density Plus devices.
                                    You can use STMicroelectronics STM32L152C-Discovery board to run this project.

     - STM32L1XX_HD(STM32L1xxxDxx): to configure the project for STM32L Ultra Low Power High-density devices.
                                    You can use STMicroelectronics STM32L152D-EVAL board to run this project.

     - STM32L1XX_XL(STM32L1xxxExx): to configure the project for STM32L Ultra Low Power XL-density devices.
                                    You can use STMicroelectronics STM32L152xE-Nucleo board to run this project.

 - Rebuild all files: Project->Rebuild all target files
 - Load project image: Debug->Start/Stop Debug Session
 - Run program: Debug->Run (F5)

@note
- Ultra Low Power Medium-density devices: - STM32L151x6xx, STM32L151x8xx, STM32L151xBxx, STM32L152x6xx,
                                            STM32L152x8xx, STM32L152xBxx, STM32L151x6xxA, STM32L151x8xxA,
                                            STM32L151xBxxA, STM32L152x6xxA, STM32L152x8xxA and STM32L152xBxxA
                                          - STM32L100x6xx, STM32L100x8xx and STM32L100xBxx
- Ultra Low Power Medium-density Plus devices: - STM32L151xCxx, STM32L152xCxx and STM32L162xCxx 
                                               - STM32L100xCxx
- Ultra Low Power High-density devices: STM32L151xDxx, STM32L152xDxx and STM32L162xDxx
- Ultra Low Power XL-density devices: STM32L151xExx, STM32L152xExx and STM32L162xExx
    
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */


