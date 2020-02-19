/*
@licstart  The following is the entire license notice for the
JavaScript code in this file.

Copyright (C) 1997-2019 by Dimitri van Heesch

This program is free software; you can redistribute it and/or modify
it under the terms of version 2 of the GNU General Public License as published by
the Free Software Foundation

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

@licend  The above is the entire license notice
for the JavaScript code in this file
*/
var NAVTREE =
[
  [ "SD_Modules", "index.html", [
    [ "Steps to configure BMM150 Interrupts", "md_bmx160__interrupt_settings.html", null ],
    [ "BMI160 sensor API", "md_bmx160__r_e_a_d_m_e.html", [
      [ "Introduction", "md_bmx160__r_e_a_d_m_e.html#autotoc_md12", null ],
      [ "Version", "md_bmx160__r_e_a_d_m_e.html#autotoc_md13", null ],
      [ "Integration details", "md_bmx160__r_e_a_d_m_e.html#autotoc_md14", null ],
      [ "File information", "md_bmx160__r_e_a_d_m_e.html#autotoc_md15", null ],
      [ "Supported sensor interface", "md_bmx160__r_e_a_d_m_e.html#autotoc_md16", null ],
      [ "Usage guide", "md_bmx160__r_e_a_d_m_e.html#autotoc_md17", [
        [ "TABLE 1.0-Macros for enabling the desired_settings", "md_bmx160__interrupt_settings.html#autotoc_md6", null ],
        [ "TABLE 1.1-Interrupt configuation settings", "md_bmx160__interrupt_settings.html#autotoc_md7", null ],
        [ "Configuring and handling BMM150 Interrupts", "md_bmx160__interrupt_settings.html#autotoc_md8", [
          [ "Example for using high threshold interrupt", "md_bmx160__interrupt_settings.html#autotoc_md9", null ],
          [ "Example for using multiple interrupts (data ready and overflow interrupts)", "md_bmx160__interrupt_settings.html#autotoc_md10", null ]
        ] ],
        [ "Initializing the sensor", "md_bmx160__r_e_a_d_m_e.html#autotoc_md18", [
          [ "Example for SPI 4-Wire", "md_bmx160__r_e_a_d_m_e.html#autotoc_md19", null ],
          [ "Example for I2C", "md_bmx160__r_e_a_d_m_e.html#autotoc_md20", null ]
        ] ],
        [ "Configuring accel and gyro sensor", "md_bmx160__r_e_a_d_m_e.html#autotoc_md21", [
          [ "Example for configuring accel and gyro sensors in normal mode", "md_bmx160__r_e_a_d_m_e.html#autotoc_md22", null ]
        ] ],
        [ "Reading sensor data", "md_bmx160__r_e_a_d_m_e.html#autotoc_md23", [
          [ "Example for reading sensor data", "md_bmx160__r_e_a_d_m_e.html#autotoc_md24", null ]
        ] ],
        [ "Setting the power mode of sensors", "md_bmx160__r_e_a_d_m_e.html#autotoc_md25", [
          [ "Example for setting power mode of accel and gyro", "md_bmx160__r_e_a_d_m_e.html#autotoc_md26", null ]
        ] ],
        [ "Reading sensor data register", "md_bmx160__r_e_a_d_m_e.html#autotoc_md27", [
          [ "Example for reading Chip Address", "md_bmx160__r_e_a_d_m_e.html#autotoc_md28", null ]
        ] ],
        [ "Writing to sensor data register", "md_bmx160__r_e_a_d_m_e.html#autotoc_md29", [
          [ "Example for writing data to any motion threshold register", "md_bmx160__r_e_a_d_m_e.html#autotoc_md30", null ]
        ] ],
        [ "Resetting the device using soft-reset", "md_bmx160__r_e_a_d_m_e.html#autotoc_md31", [
          [ "Example for writing soft-reset command to command register", "md_bmx160__r_e_a_d_m_e.html#autotoc_md32", null ]
        ] ],
        [ "Configuring interrupts for sensors", "md_bmx160__r_e_a_d_m_e.html#autotoc_md33", null ],
        [ "Configuring Any-motion Interrupt", "md_bmx160__r_e_a_d_m_e.html#autotoc_md34", [
          [ "Example for configuring Any-motion Interrupt", "md_bmx160__r_e_a_d_m_e.html#autotoc_md35", null ]
        ] ],
        [ "Configuring Flat Interrupt", "md_bmx160__r_e_a_d_m_e.html#autotoc_md36", [
          [ "Example for configuring Flat Interrupt", "md_bmx160__r_e_a_d_m_e.html#autotoc_md37", null ]
        ] ],
        [ "Configuring Step Detector Interrupt", "md_bmx160__r_e_a_d_m_e.html#autotoc_md38", [
          [ "Example for configuring Step Detector Interrupt", "md_bmx160__r_e_a_d_m_e.html#autotoc_md39", null ]
        ] ],
        [ "Configuring Step counter", "md_bmx160__r_e_a_d_m_e.html#autotoc_md40", null ],
        [ "User space", "md_bmx160__r_e_a_d_m_e.html#autotoc_md41", null ],
        [ "ISR", "md_bmx160__r_e_a_d_m_e.html#autotoc_md42", null ],
        [ "Unmapping Interrupt", "md_bmx160__r_e_a_d_m_e.html#autotoc_md43", [
          [ "Example for unmapping Step Detector Interrupt", "md_bmx160__r_e_a_d_m_e.html#autotoc_md44", null ]
        ] ],
        [ "Reading interrupt status", "md_bmx160__r_e_a_d_m_e.html#autotoc_md45", [
          [ "Example for reading interrupt status for step detector", "md_bmx160__r_e_a_d_m_e.html#autotoc_md46", null ]
        ] ],
        [ "Configuring the auxiliary sensor BMM150", "md_bmx160__r_e_a_d_m_e.html#autotoc_md47", null ],
        [ "Accessing auxiliary BMM150 with BMM150 APIs via BMI160 secondary interface.", "md_bmx160__r_e_a_d_m_e.html#autotoc_md48", null ]
      ] ],
      [ "Integration details", "md_bmx160__r_e_a_d_m_e.html#autotoc_md49", [
        [ "Initialization of auxiliary sensor BMM150", "md_bmx160__r_e_a_d_m_e.html#autotoc_md50", null ],
        [ "Wrapper functions", "md_bmx160__r_e_a_d_m_e.html#autotoc_md51", null ],
        [ "Initialization of auxiliary BMM150 in auto mode", "md_bmx160__r_e_a_d_m_e.html#autotoc_md52", null ],
        [ "Auxiliary FIFO data parsing", "md_bmx160__r_e_a_d_m_e.html#autotoc_md53", null ]
      ] ],
      [ "Self-test", "md_bmx160__r_e_a_d_m_e.html#autotoc_md54", null ],
      [ "FIFO", "md_bmx160__r_e_a_d_m_e.html#autotoc_md56", null ],
      [ "FOC and offset compensation", "md_bmx160__r_e_a_d_m_e.html#autotoc_md58", null ],
      [ "BMM150 sensor API", "md_bmx160__r_e_a_d_m_e.html#autotoc_md62", [
        [ "Introduction", "md_bmx160__r_e_a_d_m_e.html#autotoc_md63", null ],
        [ "Version", "md_bmx160__r_e_a_d_m_e.html#autotoc_md64", null ],
        [ "Integration details", "md_bmx160__r_e_a_d_m_e.html#autotoc_md65", null ],
        [ "File information", "md_bmx160__r_e_a_d_m_e.html#autotoc_md66", null ],
        [ "Supported sensor interfaces", "md_bmx160__r_e_a_d_m_e.html#autotoc_md67", null ],
        [ "Usage guide", "md_bmx160__r_e_a_d_m_e.html#autotoc_md68", [
          [ "Example for performing accel self test", "md_bmx160__r_e_a_d_m_e.html#autotoc_md55", null ],
          [ "Example for reading FIFO and extracting Gyro data in Header mode", "md_bmx160__r_e_a_d_m_e.html#autotoc_md57", null ],
          [ "Example for configuring FOC for accel and gyro", "md_bmx160__r_e_a_d_m_e.html#autotoc_md59", null ],
          [ "Example for updating the offsets manually", "md_bmx160__r_e_a_d_m_e.html#autotoc_md60", null ],
          [ "Example for updating the offsets into NVM", "md_bmx160__r_e_a_d_m_e.html#autotoc_md61", null ],
          [ "Initializing the sensor", "md_bmx160__r_e_a_d_m_e.html#autotoc_md69", [
            [ "Example for SPI 4-Wire", "md_bmx160__r_e_a_d_m_e.html#autotoc_md70", null ],
            [ "Example for I2C", "md_bmx160__r_e_a_d_m_e.html#autotoc_md71", null ]
          ] ],
          [ "Sensor Configuration settings", "md_bmx160__r_e_a_d_m_e.html#autotoc_md72", [
            [ "Setting Normal operation mode (power mode) and preset mode.", "md_bmx160__r_e_a_d_m_e.html#autotoc_md73", null ]
          ] ],
          [ "Reading sensor data", "md_bmx160__r_e_a_d_m_e.html#autotoc_md75", [
            [ "Example for reading sensor data - Fixed point version", "md_bmx160__r_e_a_d_m_e.html#autotoc_md76", null ],
            [ "Example for reading sensor data - Floating point version", "md_bmx160__r_e_a_d_m_e.html#autotoc_md77", null ]
          ] ],
          [ "Self test - Normal self test and Advanced self test", "md_bmx160__r_e_a_d_m_e.html#autotoc_md78", [
            [ "Example for performing Normal self test and Advanced self test", "md_bmx160__r_e_a_d_m_e.html#autotoc_md79", null ]
          ] ],
          [ "Interrupt settings", "md_bmx160__r_e_a_d_m_e.html#autotoc_md80", [
            [ "Example for using Data ready interrupt to read data", "md_bmx160__r_e_a_d_m_e.html#autotoc_md81", null ]
          ] ]
        ] ],
        [ "Copyright (C) 2019 Bosch Sensortec GmbH", "md_bmx160__r_e_a_d_m_e.html#autotoc_md82", null ]
      ] ]
    ] ],
    [ "CONTENTS OF THIS FILE", "md_bno055__r_e_a_d_m_e.html", [
      [ "INTRODUCTION", "md_bno055__r_e_a_d_m_e.html#autotoc_md83", null ],
      [ "VERSION", "md_bno055__r_e_a_d_m_e.html#autotoc_md84", null ],
      [ "INTEGRATION DETAILS", "md_bno055__r_e_a_d_m_e.html#autotoc_md85", null ],
      [ "Driver FILES INFORMATION", "md_bno055__r_e_a_d_m_e.html#autotoc_md86", [
        [ "bno055.h", "md_bno055__r_e_a_d_m_e.html#autotoc_md87", null ],
        [ "bno055.c", "md_bno055__r_e_a_d_m_e.html#autotoc_md88", null ],
        [ "bno055_support.c", "md_bno055__r_e_a_d_m_e.html#autotoc_md89", null ]
      ] ],
      [ "SUPPORTED SENSOR INTERFACE", "md_bno055__r_e_a_d_m_e.html#autotoc_md90", null ],
      [ "COPYRIGHT", "md_bno055__r_e_a_d_m_e.html#autotoc_md91", null ]
    ] ],
    [ "README", "md__r_e_a_d_m_e.html", null ],
    [ "Deprecated List", "deprecated.html", null ],
    [ "Modules", "modules.html", "modules" ],
    [ "Data Structures", "annotated.html", [
      [ "Data Structures", "annotated.html", "annotated_dup" ],
      [ "Data Structure Index", "classes.html", null ],
      [ "Class Hierarchy", "hierarchy.html", "hierarchy" ],
      [ "Data Fields", "functions.html", [
        [ "All", "functions.html", "functions_dup" ],
        [ "Variables", "functions_vars.html", "functions_vars" ]
      ] ]
    ] ],
    [ "Files", "files.html", [
      [ "File List", "files.html", "files_dup" ],
      [ "Globals", "globals.html", [
        [ "All", "globals.html", "globals_dup" ],
        [ "Functions", "globals_func.html", "globals_func" ],
        [ "Typedefs", "globals_type.html", null ],
        [ "Enumerations", "globals_enum.html", null ],
        [ "Enumerator", "globals_eval.html", null ],
        [ "Macros", "globals_defs.html", "globals_defs" ]
      ] ]
    ] ]
  ] ]
];

var NAVTREEINDEX =
[
"_bsx_fusion_library_8h.html",
"_bsx_library_constants_8h.html#abc70a8c5655d01bf8297cfae0dc3f81d",
"bmi160__defs_8h.html",
"bmi160__defs_8h.html#a7e2bb2061f9d1f789cb97969a0287004",
"bno055_8h.html#a00f55e3d994763331968f2203aac7dfe",
"bno055_8h.html#a3123e1a4d4cb3b113afddad709864170",
"bno055_8h.html#a5c286886642d6c339e36cf83d53651ea",
"bno055_8h.html#a8dd05b84b39eaf1ece3a08cdb1e69fd8",
"bno055_8h.html#aba65b8682ba0645b5b9ab0ea64eb4c7e",
"bno055_8h.html#ae2fa1dfc768032805ed62dd07a0890fd",
"functions_vars_l.html",
"group___b_m_m150.html#gae80f53d4692065b51a8df454404fdf05",
"md_bmx160__r_e_a_d_m_e.html#autotoc_md79",
"structbmi160__fifo__frame.html#a77484dcc21f549b073a68792489615e5",
"structbno055__quaternion__t.html#a673955c85a345a695b435abafb2c06f3",
"structjson__bouy__data__t.html#a05209206c748bf62c46f18458c162d03",
"structs_state_data.html#a6d6c4cf78ca04f10664a679a19d31b4d",
"structubx__nav__pvt__t.html#a845a1ebbcb3457bd18ac0f02faa1f47a"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';