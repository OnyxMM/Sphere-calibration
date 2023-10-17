# SphereCalib

## Introduction

Welcome to the SphereCalib project! This repository contains instructions for setting up OpenCV on a Windows 10 machine with Visual Studio 2022, specifically tailored for the SphereCalib project.

## Table of Contents

- [OpenCV Installation Guide](#opencv-installation-guide)
- [Getting Started](#getting-started)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## OpenCV Installation Guide

This section provides detailed instructions for installing OpenCV on your Windows 10 machine, which is essential for running the SphereCalib project.

### Step 1: Install Visual Studio 2022

1. Download Visual Studio 2022 Community Edition from [Visual Studio Downloads](https://visualstudio.microsoft.com/downloads/). If you already have Visual Studio 2019, that works too.

2. Start the Visual Studio Installer executable that you just downloaded and select "Desktop development with C++."

3. If you previously installed Visual Studio 2022 without C++, run the Visual Studio Installer and click "Modify."

### Step 2: Download and Install OpenCV 4.6.0

1. Download OpenCV 4.6.0 (latest stable release) for Windows from the official OpenCV website: [OpenCV Releases](https://opencv.org/ -> Library -> Releases). Click on the Windows platform, and the download will start automatically. You can also download it from [here](http://cg.elte.hu/~hajder/OpenCV/).

2. Before running the downloaded .exe file, create a new folder named "OpenCV-4.6.0" in your `C:\Users\YourUserName` directory.

3. Run the installer and extract the zip file to the newly created "OpenCV-4.6.0" folder.

### Step 3: Create a Virtual Partition

1. Open the command line and type the following command: `subst t: C:\Users\YourUserName`

2. This will create a virtual partition named "T" that contains the content of the `C:\Users\YourUserName` folder. You should be able to see the "T" partition in File Explorer, which contains the OpenCV-4.6.0 folder.

### Step 4: Add OpenCV Binaries to Your System Path (Optional)

1. Search for "Edit environment variables for your account."

2. Select "Environment Variables..."

3. Edit the "Path" variable.

4. Add a new entry with the path to the installation folder of OpenCV: `T:\OpenCV-4.6.0\opencv\build\x64\vc15\bin`

## Getting Started

Provide information on how to get started with the SphereCalib project. This may include prerequisites, how to clone the repository, and any initial setup required.

## Usage

Explain how to use SphereCalib, including code examples, configuration details, and any important usage instructions.

## Contributing

If you'd like to contribute to SphereCalib, please read our [Contributing Guidelines](CONTRIBUTING.md) to understand how to get involved, submit issues, and make pull requests.

## License

The SphereCalib project is dual-licensed under the MIT License, and both Kertesz Reka and Marki Mark hold copyright.

MIT License

Copyright (c) 2023 Marki Mark and Kertesz Reka

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS," WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT, OR OTHERWISE, ARISING FROM,
OUT OF, OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
