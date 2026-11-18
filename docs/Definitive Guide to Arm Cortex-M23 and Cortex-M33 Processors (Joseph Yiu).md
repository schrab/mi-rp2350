

C H A P T E R 

# 19 

## Digital signal processing on the cortex-M33 <u>processor</u> 

### 19.1 DSP on a microcontroller? 

Digital Signal Processing (DSP) includes a wide range of mathematically intensive algorithms. The umbrella term includes applications in audio, video, measurement, and industrial control. Increasingly, digital signal processing has become one of the key requirements in many microcontroller applications. Since the introduction of the Arm<sup>®</sup> Cortex<sup>®</sup> -M4 processor in 2010, which included a DSP extension in the Armv7-M architecture, the use of Cortex-M based systems in signal processing applications has increased significantly. 

Many of these applications are focused on audio, such as portable audio players, and, in 2019, we saw the Cortex-M4 system being used in the Amazon Alexa Voice Service (AVS).<sup>a</sup> As well as that, we have also seen Arm Cortex-M processors being used in other forms of signal processing and these include: 

- Sensor fusion applications for use in mobile phones and wearable devices 

- Image processing (e.g., the OpenMV project<sup>b</sup> utilizes a Cortex-M7 microcontroller to handle image processing) 

- Detecting and analyzing sound (e.g., ai3™––a sound detection software solution<sup>c</sup> from Audio Analytic that runs on an entry-level Cortex-M0 processor) 

- Vibration analysis<sup>d</sup> for predictive maintenance 

aAWS Press release: https://aws.amazon.com/about-aws/whats-new/2019/11/new-alexa-voice-integration-foraws-iot-core-cost-effectively-brings-alexa-voice-to-any-connected-device/. 

bThe OpenMV project: https://openmv.io/. 

> cAudio Analytic ai3TM running on a Cortex-M0 microcontroller: https://www.audioanalytic.com/the-cortexm0-challenge-part-one/. 

> dA technical presentation slide on predictive maintenance from ST Microelectronics. 

751 

Definitive Guide to Arm<sup>®</sup> Cortex<sup>®</sup> -M23 and Cortex-M33 Processors https://doi.org/10.1016/B978-0-12-820735-2.00019-6 

# 2021 Elsevier Ltd. All rights reserved. 

752 

19. Digital signal processing on the cortex-M33 processor 

TABLE 19.1 DSP processing features in Cortex-M processors. 

||Multiply-accumulate<br>(MAC), saturation<br>adjustments|DSP extension (SIMD, single-cycle<br>MAC, saturation arithmetic<br>instructions)|Helium (M-<br>profile vector<br>extension)|
|---|---|---|---|
|Cortex-M0,|–|–|–|
|Cortex-M0+,||||
|Cortex-M23||||
|Cortex-M3|Yes|–|–|
|Cortex-M4,|Yes|Yes|–|
|Cortex-M7||||
|Cortex-M33|Yes|Optional|–|
|Cortex-M55|Yes|Yes|Optional|



The various members of the Cortex-M processor family have different levels of signal processing capabilities (Table 19.1). 

The Cortex-M33 processor supports, as an optional feature, the same DSP instruction set that is in the Cortex-M4 and Cortex-M7 processors. These instructions accelerate numerical algorithms and open the door to performing real-time signal processing operations directly on the Cortex-M processors, without the need for an external digital signal processor. This chapter starts with some motivation behind the DSP extensions, and provides a simple introduction to the feature using, as an example, the dot product calculation. 

I continue with a detailed look at the Cortex-M33 instruction set and provide tips and tricks for optimizing the DSP code on this processor. Chapter 20 presents the CMSIS-DSP library–– an off-the-shelf optimized DSP library provided by Arm for the Cortex-M processors. 

### 19.2 Why use a Cortex-M <u>processor</u> for a DSP application? 

Before the arrival of modern microcontrollers with DSP capabilities, the first choice that comes to mind when looking for a processor to execute Digital Signal Processing applications on is the Digital Signal Processor, which, sharing the same acronym, is also called a DSP. The architecture of a DSP is tuned to perform the mathematical operations found in these algorithms. However, in some sense, they are crippled savants that excel at certain focused operations while struggling with some of the requirements we see in embedded applications. 

Microcontrollers, on the other hand, are general purpose and excel at control tasks: interfacing to peripherals, handling user interfaces, and general connectivity. As a result, microcontrollers have a wide range of peripherals, making it easy for them to interface to other sensors and ICs with common interfaces, such as ADC, DAC, SPI, I2C/I3C, USB, and the Ethernet. Microcontrollers also have a long history of being embedded in portable products and a greater focus on minimizing power consumption, as well as delivering excellent code density. Many legacy microcontrollers, however, are not necessarily great at performing intense mathematical algorithms because they lack registers and a proper set of instructions that support these computations. 

753 

19.2 Why use a Cortex-M processor for a DSP application? 

The recent boom in connected devices creates a need for products with both microcontroller and DSP features. This is evident in devices which handle multimedia content. They require both peripheral connectivity and a capability to handle DSP processing. Traditionally, these connected devices either have: 

- Two separate chips––a general-purpose microcontroller and a digital signal processor. 

- One chip that contains both a general-purpose processor and a digital signal processor. 

Modern Cortex-M based microcontrollers, especially those that support a DSP instruction extension, enable a wide range of applications that demand a certain level of DSP processing capabilities to run on those microcontroller systems. This has many advantages: 

- There is no need for a dedicated DSP chip on the circuit board, reducing product and design costs. 

- Embedded product developers can use a single development toolchain to develop the whole application. 

- Reduces software complexity by running all software tasks on a single processor system. 

- IoT security management features on the Cortex-M processors, such as TrustZone, can be useful for a range of signal processing applications (e.g., the handling of biometric data such as fingerprints). 

- On a Cortex-M processor you can utilize its RTOS support capabilities to handle multiple signal processing threads with very little context switching overhead. In comparison, many digital signal processors do not have dedicated RTOS support. 

Some microcontrollers have multiple Cortex-M processors, which means that one or more of the Cortex-M processors could be dedicated to DSP processing, which is useful when the DSP processing workload is heavy. The other processors in the chip could then be used for other purposes such as running communication stacks, handling user interface processing tasks, etc. Although these devices have multiple processors, software developers, when developing software for these devices, are able to use a single toolchain. With the CoreSight debug architecture, multicore debugging on microcontrollers with multiple Cortex-M processors is easier than debugging a multicore system with a general-purpose processor and a digital signal processor. This is because you are likely to need multiple debug connections to debug such systems. 

With the availability of the CMSIS-DSP library, DSP application development on Cortex-M based systems is made much easier. In contrast, it can be very hard to find experienced DSP software developers who are familiar with certain proprietary DSP architectures. Cortex-M based development boards are also widely available, making them ideal starting points for teaching/learning about digital signal processing. 

While the Cortex-M33 processor can be a good match for applications that demand some digital signal processing capabilities, it is not designed to deliver the superior signal processing performance that is achieved by many dedicated digital signal processors. In 2020, Arm announced the Cortex-M55 processor, which is designed to offer a signal processing performance which is similar to a modern, mid-range digital signal processor. To achieve this performance level, the Cortex-M55 processor supports Helium technology, an extension that includes over 150 new instructions, including many vector operations. Information about Helium is not detailed here because the Cortex-M55 processor and the Armv8.1-M architecture are beyond the scope of this book. Nevertheless, because the 

754 

19. Digital signal processing on the cortex-M33 processor 

CMSIS-DSP library is available for all Arm Cortex-M processors, an application developed for the Arm Cortex-M processors using the CMSIS-DSP library can easily migrate to a Cortex-55 based device. 

Please note that the DSP extension is optional in the Cortex-M33 processor. This means that silicon chip designers can omit the DSP extension if the target usage for a chip does not involve signal processing and if ultra-low power is a higher priority requirement. If this is the case, when the DSP extension is omitted (i.e., the SIMD or the saturate arithmetic instructions are unavailable), a few variants of the multiply-accumulate instructions and saturation adjustment instructions are still available. 

Unlike the Cortex-M33 processor, the Cortex-M23 processor does not have a DSP extension and does not have the same level of processing capabilities. It can, however, still be used for lightweight signal processing tasks, such as voice activity detection and for the analysis of signals that have relatively low sampling rates (e.g., the detection of mechanical vibration). 

### 19.3 Dot <u>product</u> example 

This section discusses the salient features of DSPs with an eye toward how they improve overall performance. There are many types of processing algorithms that are useful for signal processing. Examples of low-level algorithms include filtering functions, transformations, matrix, and vector operations. Many of these processing algorithms involve a series of multiplication and add operations (i.e., multiply-accumulate) and here I will start with some of the simple functions that involve multiply-accumulate. 

By way of example, I will look at the dot product operation that multiplies two vectors and accumulates the products, element by element (Fig. 19.1). 

Assuming that inputs x[k] and y[k] are arrays of 32-bit values and that we want to use a 64-bit representation for z. In C code, the dot product would be implemented as: 

dot product implemented in C code 

int64_t dot_product (int32_t *x, int32_t *y, int32_t N) { int32_t xx, yy, k; int64_t sum = 0; for(k = 0; k < N; k++) { xx = *x++; yy = *y++; sum += xx * yy; } return sum; } 

FIG. 19.1 Dot product mathematical representation. 

755 

19.3 Dot product example 

The dot product consists of a series of multiplications and additions. This “multiplyaccumulate” or MAC operation is at the heart of many DSP functions. 

Now, consider the execution time of this algorithm on the Cortex-M33 processor. Fetching data from memory and incrementing the pointer takes just 1 clock cycle 

xx = *x++; // 1 cycle (LDR with post increment addressing mode) 

Similarly, the next fetch also takes 1cycle 

yy = *y++; // 1 cycle 

Unlike the Cortex-M4 processor, which takes 2 clock cycles for a single load operation, the Cortex-M33 processor can be slightly faster for memory access operations. 

The multiply-accumulate operation (multiplication and addition) is of the form 32 � 32 +64 and the Cortex-M33 processor can do this with a single instruction. In an ideal situation, the multiply-accumulate operation is single cycle and operates back-to-back. However, if the input data for the multiplication operations (not including the accumulator value) was loaded into the register bank at the last executed instruction, there can be a 1cycle load-use penalty. 

The loop itself (shown in the aforementioned C code example) introduces an additional overhead. This usually involves decrementing the loop counter and then branching to the beginning of the loop. The standard loop overhead ranges from 2 to 3cycles on the Cortex-M33 processor. Thus, on the Cortex-M33, the inner loop of the dot product takes 8cycles with the execution time being data dependent: 

xx = *x++; // 1 cycle yy = *y++; // 1 cycle sum += xx * yy; // 2 cycles (1 cycle pipeline due to load-use penalty) (loop overhead) // 4 cycles including loop counter update and compare 

The whole dot product function can be rewritten in assembly code as follows: 

Inline assembly code for simple dot product 

int64_t __attribute__((naked)) dot_product (int32_t *x, int32_t *y, int32_t N) { 

__asm( " PUSH {R4-R5}\n\t" /* 2 cycle */ " MOVS R4, #0\n\t" /* 1 cycle */ " MOVS R5, #0\n\t" /* 1 cycle */ "loop: \n\t" " LDR R3 , [R0], #4\n\t" /* 1 cycle */ " LDR R12, [R1], #4\n\t" /* 1 cycle */ " SMLAL R4, R5, R3, R12\n\t" /* 2 cycles - pipeline hazard */ " SUBS R2, R2, #1\n\t" /* 1 cycle */ " BNE loop\n\t" /* 3 cycles */ 



756 

19. Digital signal processing on the cortex-M33 processor 

   - " MOVS R0, R5\n\t" 

   - " MOVS R1, R4\n\t" 

   - " POP {R4-R5}\n\t" 

- " BX LR\n\t"); 

- } 

/* 1 cycle */ /* 1 cycle */ /* 2 cycle2 */ 

- /* 3 cycle2 */ 

With a simple rescheduling of instructions, we can remove the pipeline bubble of SMLAL and reduce the inner loop to 7cycles: 

Inline assembly code for simple dot product 

int64_t __attribute__((naked)) dot_product (int32_t *x, int32_t *y, int32_t N) { __asm( " PUSH {R4-R5}\n\t" /* 2 cycle */ " MOVS R4, #0\n\t" /* 1 cycle */ " MOVS R5, #0\n\t" /* 1 cycle */ "loop: \n\t" " LDR R3 , [R0], #4\n\t" /* 1 cycle */ " LDR R12, [R1], #4\n\t" /* 1 cycle */ " SUBS R2, R2, #1\n\t" /* 1 cycle – placed between LDR and SMLAL*/ " SMLAL R4, R5, R3, R12\n\t" /* 1 cycle */ " BNE loop\n\t" /* 3 cycles */ " MOVS R0, R4\n\t" /* 1 cycle */ " MOVS R1, R5\n\t" /* 1 cycle */ " POP {R4-R5}\n\t" /* 2 cycles */ " BX LR\n\t"); /* 3 cycles */ 

} 

The loop overhead can be reduced by utilizing loop unrolling. For example, if you know that the length of the vectors is a multiple of four samples, then you can unroll the loop by a factor of 4. Computing four samples would require 15cycles. In other words, the dot product operation would only require 3.75cycles on the Cortex-M33, whereas on the Cortex-M4 processor it would require 4.75cycles per sample; and on the Cortex-M3 processor it would take from 7.75 to 11.75cycles per sample. 

Inline assembly code for simple dot product with loop unrolling 

|int64_t _<br>{|_attribute__((naked))|dot_product|(int32_t *x, int32_t *y, int32_t N)|
|---|---|---|---|
|__asm(<br>"|PUSH {R4-R11}\n\t"|/*|8 cycle */|
|"|MOVS R3, #0\n\t"|/*|1 cycle */|
|"|MOVS R4, #0\n\t"|/*|1 cycle */|



757 

19.4 Getting more performance by utilizing the SIMD instructions 

"loop: \n\t" 

   - " LDMIA R0 , {R5-R8}\n\t" 

   - " LDMIA R1 , {R9-R12}\n\t" 

   - " SMLAL R3, R4, R5, R9\n\t" 

   - " SMLAL R3, R4, R6, R10\n\t" 

   - " SMLAL R3, R4, R7, R11\n\t" 

   - " SMLAL R3, R4, R8, R12\n\t" 

   - " SUBS R2, R2, #4\n\t" 

   - " BNE loop\n\t" 

   - " MOVS R0, R3\n\t" 

   - " MOVS R1, R4\n\t" 

   - " POP {R4-R11}\n\t" 

- " BX LR\n\t"); 

- } 

- /* 4 cycles */ 

- /* 4 cycles */ 

- /* 1 cycle */ 

- /* 1 cycle */ 

- /* 1 cycle */ 

- /* 1 cycle */ 

- /* 1 cycle */ 

- /* 3 cycle */ 

- /* 1 cycle */ 

- /* 1 cycle */ 

- /* 8 cycles */ 

- /* 3 cycles */ 

Please note: 

- The assembly code examples are manually created: C compiler generated code can look quite different from what is illustrated here. 

- The number of clock cycles needed for the processing is dependent on the memory wait state and, in some cases, the alignment of the instructions in the memory. 

### 19.4 Getting more <u>performance</u> by utilizing the SIMD instructions 

In the dot product example in Section 19.3, the SMLAL instruction (a MAC operation) was used. The SMLAL instruction is available even when the DSP extension is not implemented. When the DSP extension is implemented, the “Single-Instruction, Multiple-Data” (SIMD) instruction feature can be utilized to accelerate the processing of 16-bit dot product operations. 

For example, if the dot product operation is based on 16-bit data instead of 32 bits, the SMLALD, a dual MAC instruction, can be used. The operation of SMLALD is illustrated in Fig. 19.2. 

With input data arrays x[ ] and y[ ] in the dot product example in Section 19.3 both based on 16-bit data, two samples can be loaded using a single load operation; resulting in the overall dual MAC operation (as shown in Fig. 19.3, excluding the loop handling overhead) taking only 3 clock cycles, or just 1.5 clock cycle per MAC. 

Utilizing the loop unrolling technique, as illustrated in Section 19.3, the average loop overhead can be reduced. Please note, the loop counter decrement value changes from 4 to 8 as each dual MAC processes two data samples. The rewritten dot product assembly code which utilizes the SIMD operation is as follows: 

758 

19. Digital signal processing on the cortex-M33 processor 



FIG. 19.2 SMLALD instruction. 



FIG. 19.3 Reducing the number of instruction cycles using SIMD operations. 

759 

19.5 Dealing with overflows 

Inline assembly code for simple dot product with SIMD and loop unrolling 

int64_t __attribute__((naked)) dot_product (int16_t *x, int16_t *y, int32_t N) { __asm( " PUSH {R4-R11}\n\t" /* 8 cycle */ " MOVS R3, #0\n\t" /* 1 cycle */ " MOVS R4, #0\n\t" /* 1 cycle */ "loop: \n\t" " LDMIA R0 , {R5-R8}\n\t" /* 4 cycles */ " LDMIA R1 , {R9-R12}\n\t" /* 4 cycles */ " SMLALD R3, R4, R5, R9\n\t" /* 1 cycle */ " SMLALD R3, R4, R6, R10\n\t" /* 1 cycle */ " SMLALD R3, R4, R7, R11\n\t" /* 1 cycle */ " SMLALD R3, R4, R8, R12\n\t" /* 1 cycle */ " SUBS R2, R2, #8\n\t" /* 1 cycle */ " BNE loop\n\t" /* 3 cycles */ " MOVS R0, R3\n\t" /* 1 cycle */ " MOVS R1, R4\n\t" /* 1 cycle */ " POP {R4-R11}\n\t" /* 8 cycles */ " BX LR\n\t"); /* 3 cycles */ } 

In addition to undertaking dot product calculations, the SIMD dual MAC can also be deployed to many other DSP processing tasks, e.g., handling the finite impulse response (FIR) filter. For example, each of the dotted rectangles in the FIR filter shown in Fig. 19.4 can be implemented as a dual MAC. 

### 19.5 Dealing with overflows 

In the dot product example in Section 19.3, the input data is 32-bit wide and the accumulator is 64-bit wide. If the input values for the MAC operation are often close to the limits of the input value range, and there are a high number of elements in the array, the accumulation operations could easily overflow because the multiplication results are 64 bits. 

To solve this problem, many traditional DSPs use a dedicated accumulator register which is wider than the width of the multiplication results. For example, in some traditional DSPs (e.g., the SHARC processor from Analog Devices), a dedicated accumulator register with a width of 80 bits is available so that this problem does not occur. 

The Cortex-M33 processor does not, however, have a dedicated accumulator register. To avoid the aforementioned issue, an application developer could, therefore, simply restrict the input data value ranges to reduce the chance of the accumulator overflowing. Thankfully, in most signal processing applications, the input source data values, such as those for audio signals, are only 16–24 bits in size. If the data processing is carried out in a fixed-point format, the input data values can be scaled accordingly to reduce the chance of an overflow. 

760 

19. Digital signal processing on the cortex-M33 processor 





FIG. 19.4 Use of dual MAC in FIR filter. 

In some instances, it is impractical to scale input data to prevent an overflow from happening. For example, if we want to utilize the SIMD to maximize processing performance, we cannot do so as the input data values are already restricted to 16 bits (�32768 to +36767 for integer data). To help overcome this problem, the DSP extension of the Cortex-M33 processor supports saturation arithmetic. Instead of having the calculation results wrapping around at the overflow values, which is standard behavior for two’s complement arithmetic, saturation arithmetic operations limit the operation results to the maximum (e.g., +32767 for 16-bit value) or minimum (e.g., �32768 for 16-bit value). Consider the waveforms shown in Fig. 19.5. In this example, the wave forms are represented as 16-bit integers and are limited to the range �32768 to +32767. The top plot shows the ideal result, which exceeds the allowable range. The middle plot shows how the result wraps when using the standard two’s complement addition. The bottom plot shows the result with saturation. The signals are slightly clipped but are still recognizable as sine waves. 

Basic saturating arithmetic operations are supported in the DSP extension of Cortex-M33 processors, other Armv8-M Mainline processors, Cortex-M4 and Cortex-M7 processors. The processor provides saturating addition and subtraction instructions but no saturating MAC instructions. To perform a saturating MAC instruction, you have to separately multiply the two values and then perform a saturating addition. This takes one more clock cycle. 

761 

19.6 Introduction to data types for signal processing 



<!-- Start of picture text -->
x 104 Original waveform<br>4<br>2<br>0<br>-2<br>-4<br>0 100 200 300 400 500 600 700 800 900 1000<br>x 104 Result wrapped to 16 bits<br>4<br>2<br>0<br>-2<br>-4<br>0 100 200 300 400 500 600 700 800 900 1000<br>x 104 Result with clipping<br>4<br>2<br>0<br>-2<br>-4<br>0 100 200 300 400 500 600 700 800 900 1000<br><!-- End of picture text -->

FIG. 19.5 The effects of processing with and without saturation. The top plot shows the ideal result of processing, but it exceeds the allowable 16-bit range. The middle plot is the result wrapped to 16 bits and has severe distortion. The bottom plot is the result saturated to the allowable range and has mild distortion. 

### 19.6 Introduction to data types for signal <u>processing</u> 

### 19.6.1 Why we need the fixed-point data format 

Before we start going into the details of signal processing, we need to understand how signal values are represented in embedded processor systems. Various data types can be used for signal processing: 

762 

19. Digital signal processing on the cortex-M33 processor 

- Integers of various sizes 

- Floating-point values (single precision, double precision, etc.) 

- Fixed point values of various sizes 

In many signal processing applications, because the floating-point data format provides a very wide dynamic range of signal values, the floating-point support enables signal processing algorithms to be easily implemented. However, many traditional embedded processors (including some legacy Digital Signal Processors) do not support Floating-point Unit (FPU) hardware and rely on software emulation to handle floating-point calculations. However, software emulation can be very slow (in the magnitude of up to 10 times slower). At the same time, there is often a need for presenting fractional data values. While using integers with additional scaling ratios can work, it can be error prone when transferring values between different data types. 

To solve this problem, the fixed-point format was introduced to enable fractional values to be handled with integer operations. Many signal processing algorithms were developed using fixed-point format(s), which can use integer processing instructions to handle the computation––with a certain level of compromise in the signal value’s dynamic range. However, fixed-point computations can, potentially, achieve a higher performance compared to floating-point computations because many embedded processors require multiple clock cycles to execute a floating-point calculation instruction. In comparison, integer processing instructions usually take a single clock cycle. 

### 19.6.2 Fractional arithmetic 

Fractional data types are commonly used in signal processing and are unfamiliar to many software programmers. Accordingly, I have spent a bit of time introducing them and detailing their benefits in this section. 

Fixed-point data operations use regular integer data types (8-bit, 16-bit, 32-bit, etc.) and divide the bit fields into multiple parts (Fig. 19.6). Typically, these data values are signed, so the MSB (Most Significant Bit) is used for the sign bit in most cases. The rest of the bits in the data is divided into integer and fraction parts. The bit position for the integer and fraction part partitioning is called the radix point. 

The choice of radix point is application dependent. It is possible to omit the integer portion and just have the sign bit and fractional part (Table 19.2). This arrangement is quite common for a range of embedded applications. 

It is possible to use other arrangements for the partitioning of integer bit and fraction bit fields. Forexample,with8-bitsigneddata,youcanhavethefollowingfixed-pointdatatypes(Table19.3). 

An N-bit signed integer using a standard two’s complement representation represents values in the range [�2<sup>(N�1)</sup> ,2<sup>(N�1)</sup> � 1]. A fractional N-bit integer has an implied division 



FIG. 19.6 A generalized fixed-point data format representation. 

763 

19.6 Introduction to data types for signal processing 

TABLE 19.2 Common fixed-point data types. 

|Common fixed-point<br>data types|Data type definition<br>in programming|Number of<br>sign bit|Number of<br>integer bits|Number of<br>fraction bits|
|---|---|---|---|---|
|q0.7 (often refers as q7)|q7_t|1|0|7|
|q0.15 (often refers as q15)|q15_t|1|0|15|
|q0.31 (often refers as q31)|q31_t|1|0|31|



TABLE 19.3 Various radix point arrangements are possible. 

|8-bit fixed-point data types|Number of sign bit|Number of integer bits|Number of fraction bits|
|---|---|---|---|
|q0.7|1|0|7|
|q1.6|1|1|6|
|q2.5|1|2|5|
|q3.4|1|3|4|
|q4.3|1|4|3|
|q5.5|1|5|2|
|q6.7|1|6|1|
|q7.0 (same as signed integer data)|1|7|0|



by 2<sup>(N�1)</sup> and represents values in the range [�1,1 � 2<sup>�(N�1)</sup> ]. If I is the integer value then F ¼ <u>I</u> 2ð<sup>N�1</sup> Þ<sup>isthecorresponding fractionalvalue.8-bitfractionalvaluesare intherange</sup> 

|�2<sup>7</sup><br>2<sup>7 ,</sup><br>�<br>mon signed v|2<sup>7 </sup>�1<br>2<sup>7</sup><br>alues|�<br>or �1, 1<br>�<br> in two’s co|�2<sup>�7</sup><br>�<br>:<br>mplemen|
|---|---|---|---|
|Maximum|¼|01111111|¼1�2<sup>�7</sup>|
|Smallest positive|¼|00000001|¼2<sup>�7</sup>|
|Zero|¼|00000000|¼0|
|Smallest negative|¼|11111111|¼�2<sup>�7</sup>|
|Minimum|¼|10000000|¼�1|



In binary, some common signed values in two’s complement are represented as: 

Similarly, 16-bit fractional values are in the range 



764 

19. Digital signal processing on the cortex-M33 processor 

Note that the ranges of values are almost identical for the 8- and 16-bit fractional representations. This simplifies scaling in mathematical algorithms because instead of having to keep in mind a particular integer range, you simply have to remember �1 to about +1. 

You may be wondering why 8-bit fractional values are named “q7_t” rather than “q8_t”. The reason is that there is an implied sign bit in the representation and there are actually only 7 fractional bits. The individual bit values are: 



where S represents the sign bit. We can generalize fractional integers to include both integer and fractional bits. Qm.n refers to a fractional integer with 1 sign bit, m integer bits, and n fractional bits. For example, q0.7 is the familiar 8-bit data type (q7_t) mentioned above with 1 sign bit, no integer bits, and 7 fractional bits. q8.7 is a 16-bit fractional integer with 1 sign bit, 8 integer bits, and 7 fractional bits. The bit values are: 



The integer bits can be used as guard bits in signal processing algorithms. 

Fractional values are represented as integers and stored in integer variables or registers. Addition of fractional values is identical to integer addition. Multiplication, however, is fundamentally different. Multiplying two N-bit integers yields a 2N-bit result. If you need to truncate the result back to N bits you typically take the low N bits. 

Since fractional values are in the range [�1 +1], multiplying two values will yield a result in the same<sup>e</sup> data result range. Let us examine what happens if you multiply two N-bit fractional values: 

You end up with a 2N-bit result 



where I1I2 is standard integer multiplication. Since the result is 2N bits long and there is a factor of 2<sup>2N�2</sup> in the denominator, this represents a q1.(2N � 2) number. For example, for two 8-bit q0.7 fixed point data of 0.5 (i.e., binary 01000000), their integer multiplication result is 0x1000 (i.e., binary 0001_0000_0000_0000). As we know, the result should be 0.25 which should have been 0x2000. To turn this into a q0.(2N � 1) fractional number it would be left shifted by 1 bit. Alternatively, this can be turned into a q0.(N � 1) bit number by either: 

1. Left shifting by 1 bit and taking the high N bits, or 

2. Right shifting by N � 1 bits and taking the low N bits. 

> eWell almost. The only place where this falls apart is if you multiply (�1) � (�1) and obtain +1. This value is technically one LSB out of the allowable range and needs to be truncated to the largest allowable positive value. 

765 

19.7 Cortex-M33 DSP instructions 



<!-- Start of picture text -->
32-bit<br>16-bit 16-bit<br>8-bit 8-bit 8-bit 8-bit<br><!-- End of picture text -->

###### FIG. 19.7 SIMD data. 

These operations describe a true fractional multiplication in which the result is back in the range [�1 +1]. 

The Cortex-M processors uses approach (1) for fractional multiplication, but omits the left shift by 1 bit. Conceptually, the result is scaled down by 1 bit and lies in the range [�1/2 +1/2]. You have to keep this in mind when developing algorithms and, at some point, incorporate the missing bit shift. 

### 19.6.3 SIMD data 

When the DSP extension is implemented, the Cortex-M33 processor also provides SIMD instructions that operate on packed 8- or 16-bit integers. A 32-bit register can hold either 1 � 32-bit value, 2 � 16-bit values, or 4 � 8-bit values, as illustrated below (Fig. 19.7). 

Instructions which operate on 8- or 16-bit data types are useful for processing data (e.g., video or audio data) which do not require full 32-bit precision. Software developers can utilize SIMD operations with fixed-point data types, and many of the program codes in the CMSIS-DSP library are already optimized in this way. 

To use the SIMD instructions in C code, you load values into int32_t variables and then invoke the corresponding SIMD intrinsic instructions. 

### 19.7 Cortex-M33 DSP instructions 

The instructions available in the DSP extension for the Cortex-M33 processor is identical to those in the Cortex-M4 and Cortex-M7 processors. These instructions operate on data in the regular register bank: Cortex-M processors have a core register set containing sixteen 32-bit registers. The bottom 13 registers, R0–R12, are general purpose and can hold intermediate variables, pointers, function arguments, and return results. The upper three registers are reserved for special purposes. 

R0 to R12 – General purpose 

R13 – Stack pointer [reserved] R14 – Link register [reserved] R15 – Program counter [reserved] 

When optimizing code, you need to keep in mind that the register set can only hold 13 intermediate values. If you exceed this, the compiler will need to store intermediate values on the stack causing the application’s performance to slowdown. Consider the dot product C code example from Section 19.3. The registers required are: 

x - pointer 

y - pointer 

#### 766 

19. Digital signal processing on the cortex-M33 processor 

xx – 32-bit integer 

yy – 32-bit integer 

z – 64-bit integer [2 registers required] 

k – loop counter 

In total, this function needs seven registers. The dot product is quite basic and already half of the registers are used. 

The optional floating-point unit (FPU) on the Cortex-M33 processor has its own set of registers. This extension register file contains 32 single-precision registers (32 bits each) labeled S0–S31. One obvious advantage of floating-point code is that you have access to these additional 32 registers. R0–R12 are still available to hold integer variables (pointers, counters, etc.), yielding a total of 45 registers that can be used by the C compiler. We will see later on that floating-point operations are typically slower than integer ones, but the extra registers can lead to more efficient code, especially for more complicated functions such as Fast Fourier Transforms (FFTs). Savvy C compilers make use of the floating-point registers when performing integer arithmetic. They store intermediate results to the floating-point register file, rather than to the stack, since register-to-register transfers take 1cycle as opposed to register-to-memory transfers which may, if the memory access has wait states, take multiple clock cycles. 

To use the Cortex-M33 DSP instructions, start by including the main CMSIS file core_cm33.h (this should be pulled in automatically when “including” the device header file). This defines a variety of integer, floating-point, and fractional data types as shown below: Signed Integers 

int8_t 8-bit int16_t 16-bit int32_t 32-bit int64_t 64-bit 

Unsigned Integers 

uint8_t 8-bit uint16_t 16-bit uint32_t 32-bit uint64_t 64-bit 

Floating-Point 

float32_t single precision 32-bit float64_t double precision 64-bit 

##### Fractional 

q7_t 8-bit q15_t 16-bit q31_t 32-bit q63_t 64-bit 

767 

19.7 Cortex-M33 DSP instructions 

### 19.7.1 Load and store instructions 

Loading and storing 32-bit data values can be accomplished with standard C constructs. On the Cortex-M33 processor, each load or store instruction takes 1cycle to execute. However, the read data is fetched into the processor in the third stage of pipeline. If the subsequent instruction processes the read data immediately in the second stage of the pipeline, the pipeline would have to be stalled for 1cycle. However, this pipeline stall can be avoided by scheduling other useful instructions in between the data load and the data processing. 

Please note, this pipeline characteristic is different from the Cortex-M3 and the Cortex-M4 processors where a single data load instruction takes two cycles and where subsequent loads or stores take 1cycle. When working with these processors, whenever possible, group loads and stores together to take advantage of this situation, i.e., saving 1cycle. 

To load or store packed SIMD data, define int32_t variables to hold the data. Then perform the loads and stores using the __SIMD32 macros supplied in the CMSIS library. For example, to load four 8-bit values in one instruction use 

q7_t *pSrc *pDst; int32_t x; 

x = *__SIMD32(pSrc)++; 

This also increments pSrc by a full 32-bit word (so that it points to the next group of four 8-bit values). To store the data back to memory, use 

*_SIMD32(pDst)++ = x; 

The macro also applies to packed 16-bit data 

q15_t *pSrc *pDst; int32_t x; x = *__SIMD32(pSrc)++; 

*__SIMD32(pDst)++ = x; 

### 19.7.2 Arithmetic instructions 

This section describes the Cortex-M arithmetic operations which are most frequently used in DSP algorithms. The goal is not to cover each and every Cortex-M arithmetic instruction but rather to highlight the ones that are most frequently used in practice. In fact, I only cover a fraction of the available commands. The focus is on: 

|Signed data|[Ignore unsigned]|
|---|---|
|Floating-point||
|Fractional integers|[Ignore standard integer math]|
|Sufficient precision|[32-bit or 64-bit accumulators]|



I also ignore fractional operations with rounding, add/subtract variants, carry bits, and variants based on top or bottom placement of 16-bit words within a 32-bit register. I have 

768 

19. Digital signal processing on the cortex-M33 processor 

ignored these instructions because they are less frequently used and once the content of this section is understood, it should be easy to apply the other variants. 

The instructions vary on how they are invoked by the C compiler. In some cases, the compiler determines the correct instruction to use based on standard C code. For example, fractional addition is performed using: 

z = x + y; 

In other cases, you have to use an idiom. This is a predefined snippet of C code that the compiler recognizes and maps to the appropriate single instruction. For example, to swap bytes 0 and 1, and 2 and 3 of a 32-bit word, use the idiom 

(((x&0xff )<<8)¦((x&0xff00)>>8)¦((x&0xff000000)>>8)¦((x&0x00ff0000)<<8)); 

The compiler recognizes this and maps it to a single REV16 instruction. 

And finally, in some cases there is no C construct that maps to the underlying instruction and the instruction can only be invoked via an intrinsic. For example, to carry out a saturating 32-bit addition use 

z = __QADD(x, y); 

In general, it is better to use idioms because they are standard C constructs and provide portability across processors and compilers. The idioms described in this chapter apply to Keil<sup>®</sup> Microcontroller Development Kit (MDK) and the idioms map to precisely 1 Cortex-M instruction. Some compilers do not provide full support for idioms and may yield several instructions. Check your compiler documentation to see which idioms are supported and how they map to Cortex-M instructions. Alternatively, a range of portable intrinsic functions are available in the CMSIS-CORE. Information appertaining to this is available in the CMSIS reference page: https://arm-software.github.io/CMSIS_5/Core/html/group__intrinsic__SIMD__gr.html. 

##### 19.7.2.1 32-bit integer instructions 

ADD—32-bit addition 

Standard 32-bit addition without saturation––may result in an overflow situation. It supports both int32_t and q31_t data types. 

Processor support: All Cortex-M processors [Takes 1cycle]. C code example 

q31_t x, y, z; 

z = x + y; 

###### SUB—32-bit subtraction 

Standard 32-bit subtraction without saturation––may result in an overflow situation. It supports both int32_t and q31_t data types. 

Processor support: All Cortex-M processors [Takes 1cycle]. C code example 

q31_t x, y, z; z = x - y; 

769 

19.7 Cortex-M33 DSP instructions 

###### SMULL—Long signed multiply 

Multiplies two 32-bit integers and returns a 64-bit result. This is useful for computing products of fractional data while maintaining high precision. 

Processor support: Cortex-M3 [Takes 3–7cycles] and Cortex-M4, Cortex-M7, Cortex-M33, Cortex-M55 [Takes 1cycle]. 

C code example 

int32_t x, y; int64_t z; z = (int64_t) x * y; 

SMLAL—Long signed multiply accumulate 

Multiplies two 32-bit integers and adds the 64-bit result to a 64-bit accumulator. This is useful for computing MACs of fractional data and maintaining high precision. 

Processor support: Cortex-M3 [3–7cycles] and Cortex-M4, Cortex-M7, Cortex-M33, Cortex-M55 [1cycle]. C code example 

int32_t x, y; int64_t acc; acc += (int64_t) x * y; 

###### SSAT—Signed saturation 

Saturates a signed x integer to a specified bit position B. The result is saturated to the range 

�2<sup>B�1</sup> � x � 2<sup>B�1</sup> � 1 

where B ¼ 1, 2, …, 32. The instruction is available in C code only via the intrinsic: 

int32_t __SSAT(int32_t x, uint32_t B) 

Processor support: All Armv7-M and Armv8-M Mainline processors [Takes 1cycle]. C code example 

int32_t x, y; y = __SSAT(x, 16); // Saturate to 16-bit precision. 

###### SMMUL—32-bit multiply returning 32-most-significant-bits 

Fractional q31_t multiplication (if the result is left shifted by 1 bit). Multiplies two 32-bit integers, generates a 64-bit result, and then returns the high 32 bits of the result. 

Processor support: Cortex-M4, Cortex-M7, and Armv8-M Mainline processors with the DSP extension [Takes 1cycle]. 

The instruction is available in C code via the following idiom: 

(int32_t) (((int64_t) x * y) >> 32) 

C code example 

// Performs a true fractional multiplication but loses the LSB of the result int32_t x, y, z; 

770 

19. Digital signal processing on the cortex-M33 processor 

z = (int32_t) (((int64_t) x * y) >> 32); 

z <<= 1; 

A related instruction is SMULLR, which rounds the 64-bit result of multiplication rather than simply truncating. The rounded instruction provides slightly higher precision. SMULLR is accessed via the idiom: 

(int32_t) (((int64_t) x * y + 0x80000000LL) >> 32) 

SMMLA—32-bit multiply with 32-most-significant-bit accumulate 

Fractional q31_t multiply accumulate. Multiplies two 32-bit integers, generates a 64-bit result, and adds the high bits of the result to a 32-bit accumulator. 

Processor support: Cortex-M4, Cortex-M7, and Armv8-M Mainline processors with the DSP extension [Takes 1cycle]. 

The instruction is available in C code via the following idiom: 

(int32_t) (((int64_t) x * y + ((int64_t) acc << 32)) >> 32); 

C code example 

// Performs a true fractional MAC int32_t x, y, acc; 

acc = (int32_t) (((int64_t) x * y + ((int64_t) acc << 32)) >> 32); acc <<= 1; 

Related instructions are SMMLAR which includes rounding and SMMLS which performs subtraction rather than addition. 

###### QADD—32-bit saturating addition 

Adds two signed integers (or fractional integers) and saturates the result. Positive values are saturated to 0x7FFFFFFF and negative values are saturated to 0x80000000; wrap around does not occur. 

The instruction is available in C code only via the intrinsic: 

int32_t __QADD(int32_t x, uint32_t y) 

Processor support: Cortex-M4, Cortex-M7, and Armv8-M Mainline processors with the DSP extension [Takes 1cycle] 

C code example 

int32_t x, y, z; z = __QADD(x, y); Related instruction: QSUB—32-bit saturating subtraction 

###### SDIV—32-bit division 

Divides two 32-bit values and returns a 32-bit result. 

Processor support: Armv7-M and Armv8-M processors [Takes 2–12cycles] 

771 

19.7 Cortex-M33 DSP instructions 

C code example 

int32_t x, y, z; z = x / y; 

##### 19.7.2.2 16-bit integer instructions 

SADD16—Dual 16-bit addition 

Adds two 16-bit values using SIMD. If overflow occurs then the result wraps around. Processor support: Cortex-M4, Cortex-M7, and Armv8-M Mainline processors with the DSP extension [Takes 1cycle] 

C code example 

int32_t x, y, z; z = __SADD16(x, y); 

Related instructions 

SSUB16—Dual 16-bit subtraction 

###### QADD16—Dual 16-bit saturating addition 

Adds two 16-bit values using SIMD. If an overflow occurs then the result is saturated. Positive values are saturated to 0x7FFF and negative values are saturated to 0x8000. 

Processor support: Cortex-M4, Cortex-M7, and Armv8-M Mainline processors with the DSP extension [Takes 1cycle] 

C code example 

int32_t x, y, z; z = __QADD16(x, y); 

Related instructions 

QSUB16—Dual 16-bit saturating subtraction 

SSAT16—Dual 16-bit saturate 

Saturates two signed 16-bit values to bit position B. The resulting values are saturated to the range 

�2<sup>B�1</sup> � x � 2<sup>B�1</sup> � 1 

where B ¼ 1, 2, … 16. The instruction is available in C code only via the intrinsic: 

int32_t __ssat16(int32_t x, uint32_t B) 

Processor support: Cortex-M4, Cortex-M7, and Armv8-M Mainline processors with the DSP extension [Takes 1cycle] 

C code example 

int32_t x, y; y = __SSAT16(x, 12); // Saturate to bit 12. 

772 

19. Digital signal processing on the cortex-M33 processor 

SMLABB—Q setting 16-bit signed multiply with 32-bit accumulate, bottom by bottom 

Multiplies the low 16 bits of two registers and adds the result to a 32-bit accumulator. If an overflow occurs during the addition then the result will wrap. 

Processor Support: Cortex-M4, Cortex-M7, and Armv8-M Mainline processors with the DSP extension [Takes 1cycle] 

The instruction is available in C code using standard arithmetic operations: 

int16_t x, y; int32_t acc1, acc2; acc2 = acc1 + (x * y); 

SMLAD—Q setting dual 16-bit signed multiply with single 32-bit accumulator 

Multiplies two signed 16-bit values and adds both results to a 32-bit accumulator. (top * top)+(bottom * bottom). If an overflow occurs during the addition then the result will wrap. This is the SIMD version of SMLABB. 

Processor Support: Cortex-M4, Cortex-M7, and Armv8-M Mainline processors with the DSP extension [Takes 1cycle] 

The instruction is available in C code using the intrinsic: 

sum = __SMLAD(x, y, z) 

Conceptually, the intrinsic performs the operations: 

sum = z + ((short)(x>>16) * (short)(y>>16)) + ((short)x * (short)y) 

Related instructions are. 

SMLADX—Dual 16-bit signed multiply add with 32-bit accumulate. (top * bottom) +(bottom * top). 

SMLALBB—16-bit signed multiply with 64-bit accumulate, bottom by bottom 

Multiplies the low 16 bits of two registers and adds the result to a 64-bit accumulator. Processor Support: Cortex-M4, Cortex-M7, and Armv8-M Mainline processors with the DSP extension [Takes 1cycle] 

The instruction is available in C code using standard arithmetic operations: 

int16_t x, y; int64_t acc1, acc2; acc2 = acc1 + (x * y); 

See also SMLALBT, SMLALTB, SMLALTT. 

SMLALD—Dual 16-bit signed multiply with single 64-bit accumulator 

Performs two 16-bit multiplications and adds both results to a 64-bit accumulator. (top * top)+(bottom * bottom). If an overflow occurs during the accumulation then the result wraps. The instruction is only available in C code via the intrinsic: 

uint64_t __SMLALD(uint32_t val1, uint32_t val2, uint64_t val3) 

773 

19.7 Cortex-M33 DSP instructions 

##### C code example 

// Input arguments each containing 2 packed 16-bit values // x[31:16] x[15:0], y[31:15] y[15:0] uint32_t x, y; // 64-bit accumulator uint64_t acc; 

// Computes acc += x[31:15]*y[31:15] + x[15:0]*y[15:0] acc = __SMLALD(x, y, acc); 

Related instructions are 

SMLSLD—Dual 16-bit signed multiply subtract with 64-bit accumulate SMLALDX—Dual 16-bit signed multiply add with 64-bit accumulate. (top * bottom) +(bottom * top). 

##### 19.7.2.3 8-bit integer instructions 

SADD8—Quad 8-bit addition 

Adds four 8-bit values using SIMD. If an overflow occurs then the result wraps around. 

Processor support: Cortex-M4, Cortex-M7, and Armv8-M Mainline processors with the DSP extension [Takes 1cycle] 

C code example 

// Input arguments contain 4 8-bit values each: // x[31:24] x[23:16] x[15:8] x[7:0] // y[31:24] y[23:16] y[15:8] y[7:0] int32_t x, y; // Result also contains 4 8-bit values: // z[31:24] z[23:16] z[15:8] z[7:0] int32_t z; // Computes without saturation: // z[31:24] = x[31:24] + y[31:24] // z[25:16] = x[25:16] + y[25:16] // z[15:8] = x[15:8] + y[15:8] // z[7:0] = x[7:0] + y[7:0] z = __SADD8(x, y); 

Related instructions 

SSUB8—Quad 8-bit subtraction 

###### QADD8—Quad 8-bit saturating addition 

Adds four 8-bit values using SIMD. If an overflow occurs then the result is saturated. Positive values are saturated to 0x7F and negative values are saturated to 0x80. 

Processor support: Cortex-M4, Cortex-M7, and Armv8-M Mainline processors with the DSP extension [Takes 1cycle] 

774 

19. Digital signal processing on the cortex-M33 processor 

##### C code example 

// Input arguments contain 4 8-bit values each: // x[31:24] x[23:16] x[15:8] x[7:0] // y[31:24] y[23:16] y[15:8] y[7:0] int32_t x, y; // Result also contains 4 8-bit values: // z[31:24] z[23:16] z[15:8] z[7:0] int32_t z; // Computes with saturation: // z[31:24] = x[31:24] + y[31:24] // z[25:16] = x[25:16] + y[25:16] // z[15:8] = x[15:8] + y[15:8] // z[7:0] = x[7:0] + y[7:0] z = __QADD8(x, y); 

Related instructions 

QSUB8—Quad 8-bit saturating subtraction 

##### 19.7.2.4 Floating-point instructions 

The floating-point instructions on the Cortex-M33 processors are fairly straightforward and most of them are directly accessible via C code. The instructions execute natively if the Cortex-M33 processor includes the floating-point unit (FPU). Most instructions execute in 1cycle if the result is not used in the next instruction. The floating-point instructions are IEEE 754 standard compliant [1]. 

If the FPU is not implemented, then the instructions are emulated in software and execute much more slowly. I provide here only a quick overview of the floating-point instructions. 

VABS.F32—Floating-point absolute value 

Computes the absolute value of a floating-point value. 

float x, y; y = fabs(x); 

VADD.F32—Floating-point addition 

Adds two floating-point values. 

float x, y, z; z = x + y; 

VDIV.F32—Floating-point division 

Divides two floating-point values (multicycles). 

float x, y, z; z = x / y; 

775 

19.7 Cortex-M33 DSP instructions 

VMUL.F32—Floating-point multiplication 

Multiplies two floating-point values. 

float x, y, z; z = x * y; 

VMLA.F32—Floating-point multiply accumulate 

Multiplies two floating-point values and adds the result to a floating-point accumulator.<sup>f</sup> 

float x, y, z, acc; acc = z + (x * y); 

###### VFMA.F32—Fused floating-point multiply accumulate 

Multiplies two floating-point values and adds the result to a floating-point accumulator. This is slightly different from the standard floating-point multiply accumulate (VMLA), which performs two rounding operations; one after the multiplication and then a second one after the addition. The fused multiply accumulate maintains full precision of the multiplication result and performs a single rounding operation after the addition. This yields a slightly more precise result, with roughly half the rounding error. The main use of the fused MAC is in iterative operations such as divisions or square roots. 

float x, y, acc; acc = 0; __fmaf(x, y, acc); 

###### VNEG.F32—Floating-point negation 

Multiplies a floating-point value by �1. 

float x, y; y = -x; 

###### VSQRT.F32—Floating-point square root 

Computes the square root of a floating-point value (multicycles). 

float x, y; y = __sqrtf(x); 

###### VSUB.F32—Floating-point subtraction 

Subtracts two floating-point values. 

float x, y, z; z = x - y; 

> fI recommend not using the multiply accumulate instruction but rather separate multiplication and addition instructions because, by doing so, it is possible to achieve a 1 cycle saving. Refer to Section 19.7.3.4 for further information. 

776 

19. Digital signal processing on the cortex-M33 processor 

### 19.7.3 General Cortex-M33 optimization strategies 

This section builds upon the instruction set introduction from the previous section and describes common optimization strategies that can be applied to DSP algorithms on the Cortex-M33 processor. 

##### 19.7.3.1 Scheduling of load and store instructions 

Load or store instructions on the Cortex-M33 processor take 1cycle, but if the next instruction uses the loaded data, the pipeline could stall due to the nature of its design. To maximize performance, try scheduling other instructions between the load and the data processing instruction. For example, in the dot product example shown in Section 19.3, the loop counter update was placed between the load and the MAC (SMLAL instruction) to avoid the waste clock cycle. This optimization applies to both integer and floating-point operations. 

##### 19.7.3.2 Examine the intermediate assembly code 

The underlying DSP algorithm may appear straightforward and easy to optimize, but compilers can and do get confused when setting it up. The way forward is to double check the intermediate assembly output of the C compiler to make sure that that the proper assembly instructions are being used. Also, check the intermediate code to see whether the registers are being properly used or whether intermediate results are being stored on the stack. If something does not look right, double check the compiler settings or refer to the compiler documentation. 

If you are using Keil MDK, you can enable the generation of intermediate assembly output files by adjusting the project settings as follows: go to the “Listings” tab of the target options window and check “Assembly Listing”. Then rebuild the project. 

##### 19.7.3.3 Enable optimization 

This may seem obvious, but it is worth mentioning. The code generated by a compiler varies greatly between debug mode and optimization mode. The code generated in debug mode is made for ease of debugging and not for the speed of execution. 

Compilers also provide various levels of optimization. In Arm Compiler 6, which is used in the Keil MDK, the options provided are listed in Table 19.4. 

For the best performance, a high optimization level, e.g., -O3 or -Ofast is typically used. However, I have found that when code is carefully written to schedule instructions in a certain way, using the “–O3” or the high optimization levels may result in a reordering of instructions––which is not, obviously, what you want. Accordingly, you may have to experiment with the optimization level options to determine what is the best performance for a specific algorithm. 

##### 19.7.3.4 Performance considerations in MAC instructions 

Conceptually, the MAC instruction computes a multiplication followed immediately by an addition. Since the addition requires the result of the multiplication, the overall operation of the MAC can take multiple cycles. However, back-to-back integer MAC operations are possible providing that the result of the MAC is not being used as an input for the multiplication 

777 

19.7 Cortex-M33 DSP instructions 

TABLE 19.4 Optimization levels available in Arm Compiler 6. 

|Level|Description|
|---|---|
|-O0|Turns off most optimizations. When debugging is enabled, this option generates code that directly maps<br>to the source code and can, therefore, result in a significantly large program image. This optimization<br>level is not, therefore, recommended for general debugging.|
|-O1|Restricted Optimization. When debugging is enabled, this option selects a good compromise between<br>image size, performance and the quality of the debug view. This is currently the recommended level for<br>source level debugging.|
|-O2|High optimization for speed. Code size will increase due to loop unrolling and function inlining. Also, the<br>debug view might be less satisfactory because the mapping of object code to source code is not always that<br>clear.|
|-O3|Very high optimization for speed. At this optimization the debug visibility is generally poorer.|
|-Ofast|Enables all optimizations from level 3, including those optimizations that are performed with the fast<br>math (-ffp-mode¼fast armclang option). This level also performs other aggressive optimizations that<br>might violate strict compliance with language standards.|
|-Omax|Maximum optimization. Specifically targets performance optimization. Enables all the optimizations<br>from fast level, together with other aggressive optimizations. Link Time Optimization (LTO) is enabled at<br>this level. This level is not, by default, available in the Keil MDK project option as the generated code is not<br>guaranteed to be full standards-compliant. If this level of optimization is needed, you need to manually<br>add this option in the “Misc control” field.|
|-Os|Performs optimizations to reduce code size––balancing code size against code speed.|
|-Oz|Performs optimizations to minimize image size.|



stage. In other words, the register(s) for holding the accumulator result is solely used by the MAC at the addition stage. 

For floating-point MAC operations, the cycle count for the MAC instruction (VMLA.F32/ VFMA.F32) is 3cycles. By splitting the MAC into separate multiplication (VMUL.F32) and addition (VADD.F32) parts, and by properly scheduling the instructions, by, for example, interleaving two sets of MAC operation sequences, it is possible to perform a floating-point MAC in 2cycles. When optimizing for speed for the Cortex-M33 processor, it is good practice to avoid the floating-point MAC and to only use separate arithmetic operations. The penalty is an increase in code size and not being able to take advantage of the slightly better accuracy in the fused MAC; but the increase in code size is usually negligible and, in most cases, has no impact on the results. 

##### 19.7.3.5 Loop unrolling 

The Cortex-M33 has a 2 or 3cycle overhead per loop iteration (3cycles if the branch target is an unaligned 32-bit instruction). Unrolling a loop by a factor of N effectively reduces the loop overhead to 2/N or 3/Ncycles per iteration. This can be a considerable saving, especially if the inner loop consists of only a few instructions. 

You can either manually unroll a loop by repeating a set of instructions or by having the compiler do it for you. The Keil MDK compiler supports pragmas, which guides the operation of the compiler. For example, to instruct the compiler to unroll a loop use, 

778 19. Digital signal processing on the cortex-M33 processor 

#pragma unroll for(i= 0; i < L; i++) { ... } 

In Arm Compiler 6, this pragma only has an effect when the optimization level is -O2 or higher. By default, Arm Compiler 6 (i.e., armclang) fully unrolls the loop whereas in Arm Compiler 5 (i.e., armcc) the loop is unrolled by a factor of 4. This pragma can be used with “for”, “while”, and “do-while” loops. Specifying #pragma unroll(N) causes the loop to be unrolled N times. 

Be sure to examine the generated code to make sure that the loop unrolling is effective. When unrolling, the key thing to watch for is register usage. Unrolling the loop too much may exceed the number of registers available and intermediate results will, thus, be stored on the stack leading to a degradation of performance. 

##### 19.7.3.6 Focus on the inner loop 

Many DSP algorithms contain multiple nested loops. The processing in the inner loop is executed most frequently and should be the target of the optimization work. Savings in the inner loop are essentially multiplied by the outer loop counts. Only once the inner loop is in good shape should you consider optimizing the outer loops. Many engineers spend time optimizing nontime critical code with little performance benefits to show for it. 

##### 19.7.3.7 Inline functions 

There is some overhead associated with each function call. If your function is small and executed frequently, consider inserting the code of the function in-line instead to eliminate the function call overhead. 

##### 19.7.3.8 Count registers 

The C compiler uses registers to hold intermediate results. If the compiler runs out of registers, then the results are placed on the stack. As a result, costly load and store instructions are needed to access the data on the stack. When developing an algorithm, it is good practice to start with pseudo-code to count the number of registers needed. In your count, be sure to include pointers, intermediate numerical values, and loop counters. Often, the best implementation is the one which uses the fewest intermediate registers. 

Be especially careful with register usage in fixed-point algorithms. The Cortex-M33 processor only has 13 general-purpose registers available for storing integer variables. The floatingpoint unit, on the other hand, adds 32 floating-point registers to the 13 general-purpose registers. The large number of floating-point registers makes it easy to apply loop unrolling and grouping of load and store instructions. 

##### 19.7.3.9 Use the right amount of precision 

The Cortex-M33 processor provides several multiply accumulate instructions which operate on 32-bit numbers. Some provide 64-bit results (e.g., SMLAL) and others provide 32-bit results (e.g., SMMUL). Although they both execute in a single instruction, SMLAL requires 

779 

19.8 Writing optimized DSP code for the Cortex-M33 processor 

two registers to hold the 64-bit result and may execute more slowly. This could be caused by the values being frequently transferred between the processor and the memory or could be because the registers have been used up. In general, 64-bit intermediate results are preferred, but check the generated code to make sure that the implementation is efficient. 

### 19.7.4 Instruction limitations 

The set of DSP instructions available for the Cortex-M33 processor is quite comprehensive, but there are a few gaps that make it different from some of the full feature DSP processors. These differences are: 

- Saturating fixed-point arithmetic is only available for additions and subtractions, but not for MAC instructions. For performance reasons, you will often use the fixed-point MAC and need to scale down intermediate operations to avoid overflows. 

- There is no SIMD MAC support for 8-bit values; use 16-bit SIMD MAC instead. 

### 19.8 Writing optimized DSP code for the Cortex-M33 processor 

This section shows how to use the optimization guidelines and DSP instructions to develop optimized code. I look at the Biquad filter, FFT butterfly, and FIR filter. For each, I start with generic C code and then map to the Cortex-M33 DSP instructions while applying the optimization strategies. 

### 19.8.1 Biquad filter 

A Biquad filter is a second order recursive or infinite impulse response (IIR) filter. Biquad filters are used throughout audio processing for equalization, tone controls, loudness compensation, graphic equalizers, crossovers, etc. Higher order filters can be constructed using cascades of second-order Biquad sections. In many applications, Biquad filters are the most computationally intensive part of the processing chain. Biquad filters are also similar to PID controllers used in control systems and many of the techniques developed in this section apply directly to PID controllers. 

A Biquad filter is a linear time-invariant system. When the input to the filter is a sine wave, the output is a sine wave at the same frequency but with a different magnitude and phase. The relationship between the filter’s input and output magnitudes and phases is called “frequency response”. A Biquad filter has 5 coefficients and the frequency response is determined by those coefficients. By changing coefficients, it is possible to implement lowpass, highpass, bandpass, shelf, and notch filters. For example, Fig. 19.8 shows how the magnitude response of an audio “peaking filter” varies in response to coefficient changes. Filter coefficients are typically generated by design equations<sup>g</sup> [2] or by using tools such as Matlab. 

gUseful design formulas for computing Biquad coefficients for a wide number of filter types have been provided by Robert Bristow-Johnson. See https://webaudio.github.io/Audio-EQ-Cookbook/audio-eq-cookbook.html. 

19. Digital signal processing on the cortex-M33 processor 

#### 780 



<!-- Start of picture text -->
2.5<br>2<br>1.5<br>1<br>0.5<br>0<br>1 2 3 4 5<br>10 10 10 10 10<br>Frequency (Hz)<br>)B<br>(d<br>nia<br>G<br><!-- End of picture text -->

FIG. 19.8 Typical magnitude response of a Biquad filter. This type of filter is called a “peaking filter” and boosts or cuts frequency by around 1kHz. The plots show several variants with center gains of 0.1, 0.5, 1.0, 1.5, and 2.0. 



<!-- Start of picture text -->
x[n] y[n]<br>b0<br>z -1 z -1<br>x[n-1] b1 -a1 y[n-1]<br>z -1 z -1<br>x[n-2] b2 -a2 y[n-2]<br><!-- End of picture text -->

FIG. 19.9 Direct Form I Biquad filter. This implements a second-order filter and is a building block used in higher order filters. 

The structure of a Biquad filter implemented in Direct Form I is shown in Fig. 19.9. The input x[n] arrives and feeds a delay line with two sampling stages. The boxes labeled z<sup>�1</sup> represent a one sample delay. The left-hand side of the figure is the feed-forward processing and the right-hand side is the feedback processing. Because the Biquad filter includes feedback, it is also referred to as a recursive filter. A Direct Form I Biquad filter has five coefficients, four state variables, and takes a total of five MACs per output sample. 

Since the system shown in Fig. 19.9 is linear and time invariant, we can switch the feedforward and feedback sections as shown in Fig. 19.10. With this change, the delay lines for the feedback and feed-forward sections can both take the same input and can thus be 

781 



<!-- Start of picture text -->
19.8 Writing optimized DSP code for the Cortex-M33 processor<br>x[n] y[n]<br>b0<br>z -1 z -1<br>-a1 b1<br>z -1 z -1<br>-a2 b2<br><!-- End of picture text -->

FIG. 19.10 In this figure, the feed-forward and feedback portions of the filter have been exchanged. The two delay chains receive the same input and can be combined as shown in Fig. 19.11. 



<!-- Start of picture text -->
x[n] stateA y[n]<br>b0<br>z -1<br>stateB<br>-a1 b1<br>z -1<br>stateC<br>-a2 b2<br><!-- End of picture text -->

FIG. 19.11 The Direct Form II Biquad structure. This requires five multiplications but only two delay state variables. It is preferred when undertaking floating-point processing. 

combined. This leads to the structure shown in Fig. 19.11, which is called Direct Form II. The Direct Form II filter has five coefficients, two state variables, and requires a total of five MACs per output sample. Direct Form I and Direct Form II filters are mathematically equivalent. 

The obvious advantage of the Direct Form II filter over Direct Form I is that it requires half the number of state variables. Other benefits of each structure are more subtle. If you study the Direct Form I filter you will notice that the input state variables hold delayed versions of the input. Similarly, the output state variables contain delayed versions of the output. Thus, if the gain of the filter does not exceed 1.0 then the state variables in Direct Form I will never overflow.<sup>h</sup> The state variables in the Direct Form II filter, on the other hand, have no direct relationship to the input or output of the filter. In practice, the Direct Form II state variables have a much higher dynamic range than the inputs and outputs of the filter. Thus, even if the gain of the filter does not exceed 1.0, it is possible for the state variables in Direct Form II to exceed 1.0. Because of this, Direct Form I implementations are preferred for fixed-point implementations (for better numerical behavior) while Direct Form II is preferred for floating-point implementations (because there are fewer state variables). 

> hThis is not 100% true; there are cases where you can still have overflows. Nevertheless, this rule of thumb is still useful. 

782 

19. Digital signal processing on the cortex-M33 processor 

Standard C code for computing a single stage of an implemented Biquad filter using Direct Form II is shown below. The function processes a total of blockSize samples through the filter. The input to the filter is taken from the buffer inPtr[] and the output is written to outPtr[]. Floating-point arithmetic is used. 

// b0, b1, b2, a1, and a2 are the filter coefficients. // a1 and a2 are negated. 

// stateA, stateB, and stateC represent intermediate state variables. 

for (sample = 0; sample < blockSize; sample++) 

{ stateA = *inPtr++ + a1*stateB + a2*stateC; *outPtr++ = b0*stateA + b1*stateB + b2*stateC; stateC = stateB; stateB = stateA; } 

// Persist state variables for the next call state[0] = stateB; state[1] = stateC; 

The intermediate state variables stateA, stateB, and stateC are shown in Fig. 19.11. Next, I will examine the inner loop of the function and see how many cycles are required. I will break down the operation into individual Cortex-M33 instructions: 

stateA = *inPtr++; // Data fetch [1 cycle] stateA += a1*stateB; // MAC with result used in next inst [3 cycles] stateA += a2*stateC; // MAC with result used in next inst [3 cycles] out = b0*stateA; // Mult with result used in next inst [2 cycles] out += b1*stateB; // MAC with result used in next inst [3 cycles] out += b2*stateC; // MAC with result used in next inst [3 cycles] *outPtr++ = out; // Data store [1 cycle] stateC = stateB; // Register move [1 cycle] stateB = stateA; // Register move [1 cycle] // Loop overhead [2 to 3 cycles] 

Altogether, the inner loop of the generic C code requires a total of 20–21cycles per sample to execute. 

The first step in optimizing the function is to split the MAC instructions into separate multiplications and additions. Then reordering the computation so that the result of a floatingpoint operation is not required in the next cycle. Some additional variables are used to hold the intermediate results. 

stateA = *inPtr++ // Data fetch [1 cycle] prod1 = a1*stateB; // Mult [1 cycle] prod2 = a2*stateC; // Mult [1 cycle] stateA += prod1; // Addition [1 cycle] prod4 = b1*stateB; // Mult [1 cycle] 

783 

19.8 Writing optimized DSP code for the Cortex-M33 processor 

stateA += prod2; // Add [1 cycle] out = b2*stateC; // Mult [1 cycle] prod3 = b0*stateA // Mult [1 cycle] out += prod4; // Add [1 cycle] out += prod3; // Add [1 cycle] stateC = stateB; // Register move [1 cycle] stateB = stateA; // Register move [1 cycle] *outPtr++ = out; // Data store [1 cycle] // Loop overhead [2 to 3 cycles] 

These changes reduce the inner loop of the Biquad from 20 to 15cycles. This is a step in the right direction, but there are a number of techniques that can be applied to the Biquad for further improvement: 

1. Eliminate the register moves by careful use of intermediate variables. The state variables in the structure from top to bottom are initially: stateA, stateB, stateC 

After the first output is computed, the state variables are shifted right. Instead of actually shifting the variables I will reuse them in situ using the ordering: 

stateC, stateA, stateB 

After the next iteration, the state variables are reordered as stateB, stateC, stateA 

And finally, after the fourth iteration, the variables are 

stateA, stateB, stateC 

The cycle then repeats and it has a natural period of three samples. That is, if we start with 

stateA, stateB, stateC 

then after three output samples are computed we will be back to: 

stateA, stateB, stateC 

2. Unroll the loop by a factor of 3 to reduce loop overhead. The loop overhead of 2–3cycles will be amortized over three samples. 

3. Schedule instructions to avoid stall cycles in the pipeline (applicable to the Cortex-M33 processor). If a result is produced in the third stage of the pipeline for an instruction (e.g., a memory load or a MAC) and the result is immediately used by the next instruction in the second stage of the pipeline, a stall cycle could occur. To avoid the stall, another instruction should be scheduled so that it is executed between the load/MAC instruction and the other instruction that uses the result of the load/MAC instruction. 

4. Group load and store instructions (only applicable to the Cortex-M3/M4 processors). In the above code, the load and store instructions are isolated, with each requiring 2cycles (for the Cortex-M3/M4 processors). By loading and storing several results, the second and subsequent memory accesses only takes 1cycle. 

784 

19. Digital signal processing on the cortex-M33 processor 

The resulting code is now quite a bit longer due to loop unrolling: 

in1 = *inPtr++; // Data fetch [1 cycle] in2 = *inPtr++; // Data fetch [1 cycle] in3 = *inPtr++; // Data fetch [1 cycle] prod1 = a1*stateB; // Mult [1 cycle] prod2 = a2*stateC; // Mult [1 cycle] stateA = in1+prod1; // Addition [1 cycle] prod4 = b1*stateB; // Mult [1 cycle] stateA += prod2; // Add [1 cycle] out1 = b2*stateC; // Mult [1 cycle] prod3 = b0*stateA; // Mult [1 cycle] out1 += prod4; // Add [1 cycle] out1 += prod3; // Add [1 cycle] prod1 = a1*stateA; // Mult [1 cycle] prod2 = a2*stateB; // Mult [1 cycle] stateC = in2+prod1; // Addition [1 cycle] prod4 = b1*stateA; // Mult [1 cycle] stateC += prod2; // Add [1 cycle] out2 = b2*stateB; // Mult [1 cycle] prod3 = b0*stateC; // Mult [1 cycle] out2 += prod4; // Add [1 cycle] out2 += prod3; // Add [1 cycle] prod1 = a1*stateC; // Mult [1 cycle] prod2 = a2*stateA; // Mult [1 cycle] stateB = in3+prod1; // Addition [1 cycle] prod4 = b1*stateC; // Mult [1 cycle] stateB += prod2; // Add [1 cycle] out3 = b2*stateA; // Mult [1 cycle] prod3 = b0*stateB; // Mult [1 cycle] out3 += prod4; // Add [1 cycle] out3 += prod3; // Add [1 cycle] outPtr++ = out1; // Data store [1 cycle] outPtr++ = out2; // Data store [1 cycle] outPtr++ = out3; // Data store [1 cycle] // Loop overhead [2 to 3 cycles] 

Counting cycles, we end up with 35 or 36cycles to compute three output samples or 12cycles per sample. The code presented here operates on vectors that are a multiple of three samples in length. Generally, if the total number of samples is not a multiple of 3, the code needs another stage which handles the remaining one or two samples (this is not shown in the above code). 

785 

19.8 Writing optimized DSP code for the Cortex-M33 processor 

Is it possible to optimize this further? How far can we take loop unrolling? The core arithmetic operations for the Biquad filter consists of five multiplications and four additions. These operations when properly ordered to avoid stalls take 9cycles on the Cortex-M33 processor. The memory load and store each take at best 1cycle. Putting this together, the absolute lowest number of cycles for a Biquad is 11cycles per sample. This assumes that all data loads and stores are 1cycle and there is no loop overhead. If we unroll the inner loop still further, the following would be found: 

|Unroll by|Total Cycles|Cycles/sample|
|---|---|---|
|3|36|12|
|6|71|11.833|
|9|104|11.55|
|12|137|11.41|



At some point, the processor runs out of intermediate registers to hold the input and output variables and no further gains are possible. Unrolling by three or six samples is a reasonable choice. Beyond this the gains are marginal. 

### 19.8.2 Fast Fourier Transform 

The Fast Fourier Transform (FFT) is a key signal processing algorithm that is used in frequency-domain processing, compression, and fast filtering algorithms. The FFT is actually a fast algorithm to compute the discrete Fourier transform (DFT). The DFT transforms an N- point time-domain signal x[n] into an N separate frequency component X[k],where each component is a complex value containing both magnitude and phase information. The DFT of a finite-length sequence of length N is defined as: 



where WNk is a complex value representing the kth root of unity: 



The inverse transform which converts from the frequency domain back to the time domain is nearly identical: 



Directly implementing the formulas above would require O(N<sup>2</sup> ) operations to compute all N samples of the forward or inverse transform. With the FFT we will see that this reduces to O(Nlog2N) operations. The savings can be substantial for large values of N, with the FFT 

786 

19. Digital signal processing on the cortex-M33 processor 

having made possible many new signal processing applications. The FFT was first described by Cooley and Tukey in 1965 [3] and a good general reference for FFT algorithms can be found here [4]. 

FFTs generally work best when the length N is a composite number that can be represented as a product of small factors: 

##### N ¼ N1 � N2 � N3⋯Nm 

The easiest to follow algorithms are when N is a power of 2; they are called radix-2 transforms. The FFT follows a “divide and conquer” algorithm and an N point FFT is computed using two separate N/2 point transforms, together with a few additional operations. There are two main classes of FFTs: decimation-in-time and decimation-in-frequency. Decimation-in-time algorithms compute an N-point FFT by combining N/2-point FFTs of the even and odd time-domain samples. Decimation-in-frequency algorithms are similar and compute the even and odd frequency-domain samples using two N/2-point FFTs. Both algorithms have a similar number of mathematical operations. The CMSIS-DSP library uses decimation-in-frequency algorithms and, in this chapter, I focus on these. 

The first stage of an eight-point radix-2 decimation-in-frequency FFT is shown in Fig. 19.12. The eight-point transform is computed using two separate four-point transforms. 

The multiplicative factor WkN defined earlier appears above and the values are referred to as twiddle factors. For speed, twiddle factors are precomputed and stored in an array rather than being computed in the FFT function. 

The decomposition continues and the four-point FFTs are each decomposed into two 2-point FFTs. Then at the end, four 2-point FFTs are computed. The final structure is shown in Fig. 19.13. Note that there are log28 ¼ 3 stages in the processing. 



<!-- Start of picture text -->
x[0] X[0]<br>x[1] X[2]<br>four-point<br>FFT<br>x[2] X[4]<br>x[3] X[6]<br>x[4] WN 0 X[1]<br>x[5] WN 1 X[3]<br>four-point<br>x[6] WN 2 FFT X[5]<br>x[7] WN 3 X[7]<br><!-- End of picture text -->

FIG. 19.12 First stage of an eight-point radix-2 decimation-in-frequency FFT algorithm. 



<!-- Start of picture text -->
19.8 Writing optimized DSP code for the Cortex-M33 processor 787<br>x[0] X[0]<br>x[1] WN 0 X[4]<br>x[2] WN 0 X[2]<br>x[3] WN 2 WN 0 X[6]<br>x[4] WN 0 X[1]<br>x[5] WN 1 WN 0 X[5]<br>x[6] WN 2 WN 0 X[3]<br>x[7] WN 3 WN 2 WN 0 X[7]<br>FIG. 19.13 Overall structure of an eight-point FFT. There are three stages and each stage consist of four butterflies.<br>a a'<br>b - + WNk b'<br><!-- End of picture text -->

FIG. 19.13 Overall structure of an eight-point FFT. There are three stages and each stage consist of four butterflies. 

FIG. 19.14 Single butterfly operation. 

Each stage consists of four butterfly operations and a single butterfly is illustrated in Fig. 19.14. 

Each butterfly includes a complex addition, subtraction, and multiplication. A property of the butterfly is that it can be done in place in memory. That is, it can fetch the complex values a and b, perform the operations and then place them back into the memory in the same locations where the array is. In fact, the entire FFT can be done in place with the output being generated in the same buffer that was used for the input. 

The input in Fig. 19.13 is in normal order and proceeds sequentially from x[0] to x[7]. The output after processing is scrambled and the ordering is referred to as bit-reversed order. To understand the order, write the indexes 0–7 in binary, flip the bits, and convert back to decimal: 

- 0 ➔ 000 ➔ 000 ➔ 0 

- 1 ➔ 001 ➔ 100 ➔ 4 

- 2 ➔ 010 ➔ 010 ➔ 2 

- 3 ➔ 011 ➔ 110 ➔ 6 

- 4 ➔ 100 ➔ 001 ➔ 1 

- 5 ➔ 101 ➔ 101 ➔ 5 6 ➔ 110 ➔ 011 ➔ 3 

7 ➔ 111 ➔ 111 ➔ 7 

788 

19. Digital signal processing on the cortex-M33 processor 

Bit-reversed ordering is a natural side effect of the in-place processing. Most FFT algorithms (including the ones in the CMSIS-DSP Library) provide the option of reordering the output values back into sequential order. 

Butterflies are at the heart of FFT algorithms and I will analyze and optimize the computation for a single butterfly in this section. The eight-point FFT requires 3 * 4 ¼ 12 butterflies. In general, a radix-2 FFT of length N has log2N stages each with N/2 butterflies for a total of (N/2)log2N butterflies. The decomposition into butterflies yields the O(Nlog2N) operation count for FFTs. In addition to the butterflies themselves, the FFT requires indexing to keep track of which values should be used in each stage of the algorithm. In this analysis, I will ignore this indexing overhead, but it must be accounted for in the final algorithm. 

The C code for a floating-point butterfly is shown below. The variables index1 and index2 are the array offsets for the two inputs to the butterfly. The array x[] holds interleaved data (real, imag, real, imag, etc.). 

C code implementation of a floating-point butterfly 

// Fetch two complex samples from memory [4 cycles] x1r = x[index1]; x1i = x[index1+1]; x2r = x[index2]; x2i = x[index2+1]; // Compute the sum and difference [4 cycles] sum_r = (x1r + x2r); sum_i = (x1i + x2i); diff_r = (x1r - x2r); diff_i = (x1i - x2i); // Store sum result to memory [2 cycles] x[index1] = sum_r; x[index1+1] = sum_i; // Fetch complex twiddle factor coefficients [2 cycles] twiddle_r = *twiddle++; twiddle_i = *twiddle++; // Complex multiplication of the difference [6 cycles] prod_r = diff_r * twiddle_r - diff_i * twiddle_i; prod_i = diff_r * twiddle_i + diff_i * twiddle_r; // Store back to memory [2 cycles] x[index2] = prod_r; x[index2+1] = prod_i; 

The code also shows cycle counts for the various operations and it can be seen that a single butterfly takes 20cycles on a Cortex-M33 processor. Looking more closely at the cycle count, it 

789 

19.8 Writing optimized DSP code for the Cortex-M33 processor 

can also be seen that 10cycles are due to memory accesses and 10cycles are due to arithmetic operations. To avoid stall cycles, instructions should be scheduled carefully as explained earlier in this chapter (see Section 19.8.1). Even so, on a Cortex-M33 processor a radix-2 butterfly is dominated by memory accesses and this will be true for the overall FFT algorithm. Little can be done to speed up the radix-2 FFT butterfly code. Instead, to improve performance, we need to consider higher radix algorithms. 

In a radix-2 algorithm, two complex values are operated on at a time and there is a total of log2N stages of processing. At each stage we need to load N complex values, operate on them and then store them back into the memory. In a radix-4 algorithm, four complex values are operated on at a time and there is a total of log4N stages. This cuts memory access down by a factor of 2. Higher radixes can be considered as long as we do not run out of intermediate registers. I have found that up to 

- Radix-4 butterflies using the fixed-point format, and 

- Radix-8 butterflies using the floating-point format 

can be efficiently implemented on a Cortex-M33 processor. 

A radix-4 algorithm is limited to FFT lengths, which are powers of 4: {4, 16, 64, 256, 1024, etc.}, while a radix-8 algorithm is restricted to lengths which are powers of 8: {8, 64, 512, 4096, etc.} To efficiently implement any length which is a power of 2, a mixed-radix algorithm is used. The trick is to use as many radix-8 stages as possible (they are the most efficient) and then use a single radix-2 or radix-4 stage, as needed, to achieve the desired length. Here is how the various FFT lengths break down into butterfly stages: 

|Length|Bu|tterflies|
|---|---|---|
|16|2�|8|
|32|4�|8|
|64|8�|8|
|128|2�|8�8|
|256|4�|8�8|



The FFT function in the CMSIS-DSP library uses this mixed radix approach for floatingpoint data types. For fixed-point, you must select either radix-2 or radix-4. In general, pick the radix-4 fixed-point algorithm if it supports the length you want. 

Many applications also require that inverse FFT transforms are computed. Comparing the equations for the forward and inverse DFTs, it will be seen that the inverse transform has a scale factor of (1/N) and that the sign of the exponent of the twiddle factors is inverted. This leads to two different ways of implementing inverse FFTs: 

1. Compute the forward FFT as before but use a new twiddle factor table. The new table is created using positive rather than negative exponents. This leads to twiddle factors simply being conjugated. Then divide by N. 

2. Keep the same twiddle factor table as before but modify the FFT code to negate the imaginary portion of the twiddle factor table while performing twiddle factor multiplications. Then divide by N. 

790 

19. Digital signal processing on the cortex-M33 processor 

Both approaches detailed above are somewhat inefficient. Approach (1) saves on code space but doubles the size of the twiddle factor table; Approach (2) reuses the twiddle factor table but requires more code. Another approach is to use the mathematical relationship: 



This requires conjugating the data twice. The first (inner) conjugate is carried out at the start and the second conjugate (outer) can be combined with the division by N. The real overhead of this approach compared to approach (2) is that, roughly, only the inner conjugate is required—which does not substantially increase the processing time. 

When implementing an FFT in the fixed-point format it is crucial to understand scaling and the growth of values throughout the algorithm. A butterfly performs addition and subtraction operations and it is possible for the values at the output of the butterfly to be double those at the input. In the worst-case scenario, values double at every stage and the output is N times larger than the input. Intuitively, the worst case occurs if all input values equal 1.0. This represents a DC signal and the resulting FFT is all zeros except that bin k ¼ 0 contains a value of N. In order to avoid overflow in fixed-point implementations, each butterfly stage must incorporate a scale of 0.5 as part of the addition and subtraction. This is in fact the scaling used by the fixed-point FFT functions in the CMSIS-DSP library, with the net result being that the output of the FFT is scaled down by 1/N. 

The standard FFT operates on complex data and there are variations for handling real-data. Typically, an N-point real FFT is computed using a complex N/2 point FFT, together with some additional steps. An excellent reference can be found in Ref. [5]. 

### 19.8.3 FIR filter 

The third standard DSP algorithm that I will consider is the Finite Impulse Response (FIR) filter. FIR filters occur in a number of audio, video, control algorithms, and are used for analyzing data problems. FIR filters have several useful properties compared to IIR filters (like the Biquad): 

1. FIR Filters are inherently stable. This is true for all possible coefficients. 

2. Linear phase can be achieved by making the coefficients symmetric. 

3. Simple design formulas. 

4. Well-behaved even when implemented using fixed-point. 

The operation of an FIR filter can be represented by the following formula: Let x[n] be the input to the filter at time n and let y[n] be the output. The output is computed using the difference equation: 



where h[n] are the filter coefficients. In the difference equation above, the FIR filter has N coefficients 



791 

19.8 Writing optimized DSP code for the Cortex-M33 processor 

and the output is computed using N previous input samples 



The previous input samples are called the state variables. Each output of the filter requires N multiplications and N � 1 additions. A modern DSP can compute an N point FIR filter in roughly N cycles. 

The most straightforward way of organizing the state data in the memory is to use a FIFO as shown in Fig. 19.15. When sample x[n] arrives, the previous samples x[n � 1] through x [n � N] are shifted down by one position and then x[n] is written to the buffer. Shifting data like this is very wasteful and requires N � 1 memory reads and N � 1 memory writes per input sample. 

A better way to organize the data is to use a circular buffer as shown in Fig. 19.16. The circular state index points to the oldest sample in the buffer. When sample x[n] arrives, it overwrites the oldest sample in the buffer and then circularly increments. That is, it increments in normal fashion, and if the end of the buffer is reached it wraps around back to the beginning. 

The standard C code for the FIR filter is shown below. The function is designed to operate on a block of samples and incorporates circular addressing. The outer loop is over the samples in the block while the inner loop is over the filter taps as shown in the aforementioned difference equation. 



FIG. 19.15 FIR filter implemented using a shift register. In practice this is rarely used since the state variables have to be shifted right whenever a new sample arrives. 



<!-- Start of picture text -->
x[n]<br>x[n-3] x[n-2] x[n-1] x[n] x[n-6] x[n-5] x[n-4]<br>stateIndex<br>coeffIndex<br>h[6] h[5] h[4] h[3] h[2] h[1] h[0]<br><!-- End of picture text -->

FIG. 19.16 FIR filter implemented using a circular buffer for the state variables (top). The stateIndex pointer advances to the right and then circularly wraps when it reaches the end of the buffer. The coefficients are accessed in linear order. 

792 

19. Digital signal processing on the cortex-M33 processor 

Standard C code for implementing a FIR filter by utilizing a circular buffer. The code processes a block of samples. 

// Block-based FIR filter. // N equals the length of the filter (number of taps) // blockSize equals the number of samples to process // state[] is the state variable buffer and contains the previous N // samples of the input // stateIndex points to the oldest sample in the state buffer. It // will be overwritten with the most recent input sample. // coeffs[] holds the N coefficients // inPtr and outPtr point to the input and output buffers, respectively for(sample=0;sample<blockSize;sample++) { // Copy the new sample into state[stateIndex++] = inPtr[sample] if (stateIndex >= N) stateIndex = 0; sum = 0.0f; for(i=0;i<N;i++) { sum += state[stateIndex++] * coeffs[N-i]; if (stateIndex >= N) stateIndex = 0; } outPtr[sample] = sum; } 

In order to compute each output sample, N state variables {x[n], x[n � 1], ⋯, x[n � (N � 1)]} and N coefficients {h[0], h[1], ⋯, h[N � 1]} have to be fetched from the memory. DSPs have been optimized to compute FIR filters. The state and coefficients can be fetched in parallel with the MACs and the corresponding memory pointers incremented. DSPs also have hardware support for circular addressing and can perform circular addressing without any overhead. Using these features together, a modern DSP can compute an N-point FIR filter in about N cycles. 

The Cortex-M33 processor will have difficulty implementing the FIR code efficiently. The Cortex-M33 processor does not have native support for circular addressing and the bulk of the time will be spent evaluating the “if” statement in the inner loop. A better approach is to use a FIFO for the state buffer and to shift in a block of input data. Instead of shifting the FIFO data every sample, just shift it once every block. This requires increasing the length of the state buffer by blockSize samples. This process is illustrated in Fig. 19.17 for a block size of four samples. Input data is shifted in on the right side of the block. The oldest data then appears on the left-hand side. Coefficients continue to be time flipped as shown in Fig. 19.16. 

793 

19.8 Writing optimized DSP code for the Cortex-M33 processor 



<!-- Start of picture text -->
Four new input samples are<br>shifted in<br>x[n-9] x[n-8] x[n-7] x[n-6] x[n-5] x[n-4] x[n-3] x[n-2] x[n-1] x[n]<br>h[6] h[5] h[4] h[3] h[2] h[1] h[0]<br><!-- End of picture text -->

FIG. 19.17 Work around for circular addressing. The size of the state buffer is increased by blockSize-1 samples which in this example equals three samples. 



<!-- Start of picture text -->
x[n-9] x[n-8] x[n-7] x[n-6] x[n-5] x[n-4] x[n-3] x[n-2] x[n-1] x[n]<br>h[6] h[5] h[4] h[3] h[2] h[1] h[0] Compute x[0]<br>h[6] h[5] h[4] h[3] h[2] h[1] h[0] Compute x[1]<br>Compute x[2]<br>h[6] h[5] h[4] h[3] h[2] h[1] h[0]<br>h[6] h[5] h[4] h[3] h[2] h[1] h[0] Compute x[3]<br><!-- End of picture text -->

FIG. 19.18 With the work around for circular addressing, each inner loop computation operates on continuous data. 

The coefficients in Fig. 19.17 have h[0] aligned with x[n � 3]. This is the position needed to compute the first output [n � 3]: 



To compute the next output sample, the coefficients conceptually shift over by one. This then repeats for all output samples (Fig. 19.18). By using this block-based approach with a FIFO state buffer, we are able to eliminate the costly circular addressing from the inner loop. 

To optimize the FIR filter further, there is a further need to again focus on memory access. In the standard FIR implementation, N coefficients and N state variables are accessed for each 

794 

19. Digital signal processing on the cortex-M33 processor 

output sample. The approach to take is to simultaneously compute multiple output samples and cache intermediate state variables in the registers. In the following example, four output samples are computed simultaneously.<sup>i</sup> A single coefficient is loaded and then multiplied by four state variables. This has the net effect of reducing memory access by a factor of 4. For the sake of simplicity and brevity, the code shown below only supports block sizes that are a multiple of four samples. The CMSIS-DSP library, on the other hand, is general purpose and imposes no constraints on the length of the filter or block size. Even with these simplifications, the code is still quite involved. 

Partially optimized floating-point FIR code. The example shows how the number of memory accesses can be reduced by simultaneously computing multiple results. 

/* ** Block based FIR filter. Arguments: ** numTaps - Length of the filter. Must be a multiple of 4 ** pStateBase - Points to the start of the state variable array ** pCoeffs - Points to the start of the coefficient array ** pSrc - Points to the array of input data ** pDst - Points to where the result should be written ** blockSize - Number of samples to process. Must be a multiple of 4. */ void arm_fir_f32( unsigned int numTaps, float *pStateBase, float *pCoeffs, float *pSrc, float *pDst, unsigned int blockSize) { float *pState; float *pStateEnd; float *px, *pb; float acc0, acc1, acc2, acc3; float x0, x1, x2, x3, coeff; float p0, p1, p2, p3; unsigned int tapCnt, blkCnt; /* Shift the data in the FIFO down and store the new block of input data ** at the end of the buffer. */ 

iThe number of outputs to simultaneously compute depends upon how many registers are available. In the CMSISDSP library, Q31 FIR three samples are simultaneously computed. In the floating-point version, eight samples are simultaneously computed. 

795 

19.8 Writing optimized DSP code for the Cortex-M33 processor 

/* Points to the start of the state buffer */ pState = pStateBase; /* Points ahead blockSize samples */ pStateEnd = &pStateBase[blockSize]; /* Unroll by 4 for speed */ tapCnt = numTaps >> 2u; while(tapCnt > 0u) { *pState++ = *pStateEnd++; *pState++ = *pStateEnd++; *pState++ = *pStateEnd++; *pState++ = *pStateEnd++; /* Decrement the loop counter */ tapCnt–; } /* pStateEnd points to where the new input data should be written */ pStateEnd = &pStateBase[(numTaps - 1u)]; pState = pStateBase; /* Apply loop unrolling and compute 4 output values simultaneously. * The variables acc0 ... acc3 hold output values that are being computed: * * acc0 = b[numTaps-1]*x[n-numTaps-1]+b[numTaps-2]*x[n-numTaps-2] + * b[numTaps-3]*x[n-numTaps-3]+ ... + b[0]*x[0] * acc1 = b[numTaps-1]*x[n-numTaps]+b[numTaps-2]*x[n-numTaps-1] + * b[numTaps-3]*x[n-numTaps-2] +... + b[0]*x[1] * acc2 = b[numTaps-1]*x[n-numTaps+1]+b[numTaps-2]*x[n-numTaps] + * b[numTaps-3]*x[n-numTaps-1] + ... + b[0]*x[2] * acc3 = b[numTaps-1]*x[n-numTaps+2] + b[numTaps-2]*x[n-numTaps+1] + * b[numTaps-3]*x[n-numTaps] + ... + b[0]*x[3] */ 

blkCnt = blockSize >> 2; 

/* Processing with loop unrolling. Compute 4 outputs at a time. */ while(blkCnt > 0u) { /* Copy four new input samples into the state buffer */ *pStateEnd++ = *pSrc++; *pStateEnd++ = *pSrc++; *pStateEnd++ = *pSrc++; *pStateEnd++ = *pSrc++; /* Set all accumulators to zero */ 

796 

19. Digital signal processing on the cortex-M33 processor 

acc0 = 0.0f; acc1 = 0.0f; acc2 = 0.0f; acc3 = 0.0f; /* Initialize state pointer */ px = pState; /* Initialize coeff pointer */ pb = pCoeffs; /* Read the first three samples from the state buffer: x[n-numTaps], x[n-numTaps-1], x[n-numTaps-2] */ x0 = *px++; x1 = *px++; x2 = *px++; /* Loop unrolling. Process 4 taps at a time. */ tapCnt = numTaps >> 2u; /* Loop over the number of taps. Unroll by a factor of 4. ** Repeat until we have computed numTaps-4 coefficients. */ while(tapCnt > 0u) { /* Read the b[numTaps-1] coefficient */ coeff = *(pb++); /* Read x[n-numTaps-3] sample */ x3 = *(px++); /* p = b[numTaps-1] * x[n-numTaps] */ p0 = x0 * coeff; /* p1 = b[numTaps-1] * x[n-numTaps-1] */ p1 = x1 * coeff; /* p2 = b[numTaps-1] * x[n-numTaps-2] */ p2 = x2 * coeff; /* p3 = b[numTaps-1] * x[n-numTaps-3] */ p3 = x3 * coeff; /* Accumulate */ acc0 += p0; acc1 += p1; acc2 += p2; acc3 += p3; /* Read the b[numTaps-2] coefficient */ coeff = *(pb++); 

797 

19.8 Writing optimized DSP code for the Cortex-M33 processor 

/* Read x[n-numTaps-4] sample */ x0 = *(px++); /* Perform the multiply-accumulate */ p0 = x1 * coeff; p1 = x2 * coeff; p2 = x3 * coeff; p3 = x0 * coeff; acc0 += p0; acc1 += p1; acc2 += p2; acc3 += p3; /* Read the b[numTaps-3] coefficient */ coeff = *(pb++); /* Read x[n-numTaps-5] sample */ x1 = *(px++); /* Perform the multiply-accumulates */ p0 = x2 * coeff; p1 = x3 * coeff; p2 = x0 * coeff; p3 = x1 * coeff; acc0 += p0; acc1 += p1; acc2 += p2; acc3 += p3; /* Read the b[numTaps-4] coefficient */ coeff = *(pb++); /* Read x[n-numTaps-6] sample */ x2 = *(px++); /* Perform the multiply-accumulates */ p0 = x3 * coeff; p1 = x0 * coeff; p2 = x1 * coeff; p3 = x2 * coeff; acc0 += p0; acc1 += p1; acc2 += p2; acc3 += p3; /* Read the b[numTaps-5] coefficient */ 

798 

19. Digital signal processing on the cortex-M33 processor 

coeff = *(pb++); /* Read x[n-numTaps-7] sample */ x3 = *(px++); tapCnt–; } /* Advance the state pointer to process the next * group of 4 samples */ pState = pState + 4; /* Store the 4 results in the destination buffer. */ *pDst++ = acc0; *pDst++ = acc1; *pDst++ = acc2; *pDst++ = acc3; blkCnt–; } } 

Depending on the compilation toolchain and optimization settings of the compiler, the inner loop of the floating-point FIR filter, which performs a total of 16 MACs, might take around 43cycles. The CMSIS-DSP library takes this a step further and computes eight intermediate sums, leading to a further performance gain in the inner loop. 

The q15 FIR filter functions in the CMSIS-DSP library use similar memory optimizations to the floating-point function example that I have just presented. As a further optimization, the q15 functions apply the dual 16-bit SIMD capabilities of the Cortex-M33 processor. Two q15 functions are provided: 

arm_fir_q15()—uses 64-bit intermediate accumulators and the SMLALD and SMLALDX instructions. 

arm_fir_fast_q15()—uses 32-bit intermediate accumulators and the SMLAD and SMLADX instructions. 

### References 

> [1] IEEE 754 specifications: IEEE 754–1985. https://ieeexplore.ieee.org/document/30711. IEEE 754–2008. https:// ieeexplore.ieee.org/document/4610935. 

> [2] Cookbook formulae for audio equalizer biquad filter coefficients. https://www.w3.org/2011/audio/audio-eqcookbook.html. 

- [3] J.W. Cooley, J.W. Tukey, An algorithm for the machine calculation of complex Fourier series, Math. Comput. 19 (90) (1965) 297–301. 

- [4] C.S. Burrus, T.W. Parks, DFT/FFT and Convolution Algorithms, Wiley, 1984. 

- [5] R. Matusiak, Implementing Fast Fourier Transform Algorithms of Real-Valued Sequences With the TMS320 DSP Platform, Texas Instruments Application Report SPRA291(August 2001). 



C H A P T E R 

# 20 

## Using the Arm CMSIS-DSP library 

### 20.1 Overview of the library 

The CMSIS-DSP library is a suite of common signal processing and mathematical functions that have been written for the Arm<sup>®</sup> Cortex<sup>®</sup> -M and Cortex-A processors. It has been optimized on Cortex-M processors with the DSP extension and on Cortex-A processors with the Neon (Advanced SIMD) extension. The library is freely available as part of the CMSIS release from Arm and includes all source codes. The functions in the library are divided into several categories: 

1. Basic math functions 

2. Fast math functions 

3. Complex math functions 

4. Filters 

5. Matrix functions 

6. Transforms 

7. Motor control functions 

8. Statistical functions 

9. Support functions 

10. Interpolation functions 

The library has separate functions for operating on 8-bit integers, 16-bit integers, 32-bit integers, and 32-bit floating-point values. 

The library has been optimized to take advantage of the DSP extensions found in the Cortex-M processors, including the Armv8.1-M processor with Helium technology. Although the library is available for other Cortex-M processors without a DSP extension, the functions have not been optimized for these cores; the functions operate properly but run more slowly. 

If you are using the Keil<sup>®</sup> Microcontroller Development Kit (Keil MDK), you can easily add the CMSIS-DSP library into your project. By using the Manage Run-Time Environment settings, the prebuilt library file or source codes of the CMSIS-DSP can be added into a project (Fig. 20.1). 

799 

Definitive Guide to Arm<sup>®</sup> Cortex<sup>®</sup> -M23 and Cortex-M33 Processors https://doi.org/10.1016/B978-0-12-820735-2.00020-2 

# 2021 Elsevier Ltd. All rights reserved. 

800 

20. Using the Arm CMSIS-DSP library 



<!-- Start of picture text -->
Seftare Component Sek variant Version Description<br>@ @ Board Support \V2M-MPS2IOT-Kit_ |2.00 |ARM V2M_MPS2 Board Support<br>Microcontroller<br>© cwsis% ‘Cortes Software interface Components<br>©@icoCORE Ie =) 17053.0 |CMSIS-CORECut Litafor Cortex-M.for Contes$6000.$6000 $C300 ARMvE-M, and $6300 ARMvE.1-M<br>be@ @ RTOs2NNLibnoun(API) Cc aoa f21.31.20 ||CMISIS-NNCMSIS-RTOS NeuralAPI for Cortex-M, Network LibraryC000. and $6300<br>@ © CMSIS Driver, Inified Device Drivers compliant to CMSIS-Driver Specifications<br>B®@ CompilerDevice@ Samp i ARM Compiler 1.6012 Startup‘CompilerSystem and SyitemExtensions StaSetupforfor ARNOT ARM CompilerARN5 and ARMdsice Compiler6<br>@ @ Fite System (MOK. Plus (216130 File Access on various storage devices<br>© © Guopics MOK: |] 5500 Vasotec on aaphica LCD dsas<br>1B @ Network (MOK-Plus 1) 7120 vd Networking sing Ethemetor Serial protocols<br>Communication<br>a @ use ‘MOK-Plus [216137 USB with various device classes<br><!-- End of picture text -->

FIG. 20.1 Adding the CMSIS-DSP library into a Keil MDK project. 

When integrating the CMSIS-DSP library into the project there is the option of selecting the precompiled library or the source code. Usually, the precompiled library is preferred because the precompiled libraries are optimized for use with Arm toolchains (e.g., Keil MDK) and provide the best performance. 

In addition to the prebuild libraries for Arm toolchains, such as Keil MDK, prebuilt libraries for GCC and the IAR Embedded Workbench<sup>®</sup> for Arm are also available. Multiple library files are also available for various Cortex-M processors in various FPU configurations (e.g., there are multiple versions of the library for the Cortex-M7 processor due to the availability of multiple FPU configurations), and there are also separate library files for little-endian and big-endian memory configurations. The following library files (Table 20.1) are currently available for the CMSIS installation in the Keil MDK toolchain. 

TABLE 20.1 Precompiled CMSIS-DSP library files which are available for Arm toolchains. 

|Library name|Processor|Endianness|Use DSP<br>extension|Uses FPU|
|---|---|---|---|---|
|arm_ARMv8MMLldfsp_math.lib|Cortex-M33/Cortex-M35P|Little|Yes|Yes|
|arm_ARMv8MMLld_math.lib||Little|Yes|No|
|arm_ARMv8MMLlfsp_math.lib||Little|No|Yes|
|arm_ARMv8MMLl_math.lib||Little|No|No|
|arm_ARMv8MBLl_math.lib|Cortex-M23|Little|No|No|
|arm_cortexM7lfdp_math.lib|Cortex-M7|Little|Yes|Yes (single<br>+double<br>precision)|
|arm_cortexM7bfdp_math.lib||Big|Yes|Yes (single<br>+double<br>precision)|
|arm_cortexM7lfsp_math.lib||Little|Yes|Yes (single<br>precision)|
|arm_cortexM7bfsp_math.lib||Big|Yes|Yes (single<br>precision)|



801 

20.2 Function naming convention 

TABLE 20.1 Precompiled CMSIS-DSP library files which are available for Arm toolchains—cont’d 

|Libraryname|Processor|Endianness|Use DSP<br>extension|Uses FPU|
|---|---|---|---|---|
|arm_cortexM7l_math.lib||Little|Yes|No|
|arm_cortexM7b_math.lib||Big|Yes|No|
|arm_cortexM4lf_math.lib|Cortex-M4|Little|Yes|Yes|
|arm_cortexM4bf_math.lib||Big|Yes|Yes|
|arm_cortexM4l_math.lib||Little|Yes|No|
|arm_cortexM4b_math.lib||Big|Yes|No|
|arm_cortexM3l_math.lib|Cortex-M3|Little|No|No|
|arm_cortexM3b_math.lib||Big|No|No|
|arm_cortexM0l_math.lib|Cortex-M0/Cortex-M0+|Little|No|No|
|arm_cortexM0b_math.lib||Big|No|No|



A similar range of prebuilt libraries are available for the IAR and the GCC, but the filenames are slightly different, and the precompiled libraries for the GCC are available for little endian only. If for any reason, you need to rebuild the libraries, the CMSIS-DSP library HTML documentation should be your first port of call. 

### 20.2 Function naming convention 

The functions in the library follow the naming convention: arm_OP_DATATYPE 

where OP is the operation performed and DATATYPE describes the operands 

- q7—8-bit fractional integers 

- q15—16-bit fractional integers 

- q31—32-bit fractional integers 

- f32—32-bit floating-point 

For example, some of the library function names are: 

arm_dot_prod_q7 – the dot product of 8-bit fractional integers arm_mat_add_q15 – the matrix addition of 16-bit fractional integers 

arm_fir_q31 – a FIR filter with 32-bit fractional data and coefficients arm_cfft_f32 – a complex FFT of 32-bit floating-point values 

802 

20. Using the Arm CMSIS-DSP library 

### 20.3 Getting help 

The library document is in HTML format and is located in the following folder in the CMSIS-PACK: 

CMSIS\Documentation\DSP\html 

The file index.html is the main starting point. The documentation can also be accessed on the github web page for Arm software [1]. 

### 20.4 Example 1—DTMF demodulation 

A standard touch-tone dial pad with four rows and three columns is shown in Fig. 20.2 below. Each row and column has a corresponding sine wave associated with it. When a button is pressed, the dial pad generates two sine waves; one based on the row index and another based on the column index. For example, if the number 4 is pressed then a 770Hz sine wave is generated (corresponding to the row) plus a 1209Hz sine wave (corresponding to the column). This signaling method is referred to as Dual-tone multi-frequency (DTMF) signaling and is the standard signaling method used by analog phone lines. 

In this example, we demonstrate three different methods of detecting the tones in the DTMF signal. These are: 

- FIR filter [q15] 

- FFT [q31] 

- Biquad Filter [float] 

Although I will focus on decoding one frequency, 697Hz, the example can easily be extended for decoding all seven tones. Other aspects of DTMF decoding such as setting thresholds and decision making are not covered. All of the code used in this example is shown in Section 20.4.5, Example of DTMF Code. 

The goal of the example is to show how to use various CMSIS-DSP functions and how to handle different data types. As it turns out, the Biquad filter is computationally much more 

FIG. 20.2 Keypad matrix in a DTMF signaling scheme. Each row and column has a corresponding sine wave, which is generated when a button is pressed. 



<!-- Start of picture text -->
s x &<br>ore | 1)2/3<br>mz 4/516<br>eszHz | 7 8 9<br>othe | * | QO | #<br><!-- End of picture text -->

803 

20.4 Example 1—DTMF demodulation 

efficient than the FIR filter. The reason for this is that a single Biquad stage can be used to detect a sine wave rather than a 202-point FIR filter. The Biquad is roughly 40 times more efficient than the FIR. The FFT might seem like a good choice, especially since, in a full DTMF implementation, seven frequencies will need to be checked. Still, taking all of this into account, the Biquad is still computationally more efficient than the FFT and requires much less memory as well. In practice, most DTMF receivers use the Goetzel algorithm, which is computationally very similar to the Biquad filter used in this example. 

### 20.4.1 Generating the sine wave 

In a typical DTMF application, the input data will be taken from an A/D converter. In this example, I generate the input signal using math functions. At the start of the example code, a sine wave at 697Hz or 770Hz is generated. The sine wave is first generated using a floatingpoint and is then converted to Q15 and Q31 representations. All of the processing is performed at an 8kHz sample rate, which is the standard sample rate used in telephony applications. To test the algorithms, 512 samples of the sine wave were generated with amplitude of 0.5. 

### 20.4.2 Decoding using a FIR filter 

The first approach I will use for demodulating the DTMF signal is an FIR filter. The FIR filter will have a passband centered around 697Hz and must be narrow enough to filter out the next closest frequency of 770Hz. The following MATLAB<sup>®</sup> code designs the filter. 

SR = 8000; % Sample rate FC = 697; % Center frequency of the pass band, in Hz 

NPTS = 202; 

h = fir1(NPTS-1, [0.98*FC 1.02*FC] / (SR/2), ’DC-0’ ); 

Some experimentation was needed in order to determine the correct length of the filter, so that by 770Hz there was sufficient attenuation. I found that a filter length of 201 points was sufficient. Since the CMSIS library requires an even length filter for the Q15 FIR filter function, the filter length was rounded up to 202 points. The impulse response of the designed filter is shown in Fig. 20.3 and the magnitude response is shown in Fig. 20.4. 

The filter was designed to have a gain of 1.0 in the passband. The largest coefficient of the resulting filter has a value of about 0.019. If converted to an 8-bit format (q7), the largest coefficient would only be about 2 LSBs large. The resulting filter would be severely quantized and unusable in 8-bits. At least 16-bits are required, and this example uses Q15 math. MATLAB was also used to convert the filter coefficients to the Q15 format and write them to the console window (the code below refers). The coefficients were then copied into the Cortex-M33 project. 

hq = round(h * 32768); // Quantize to Q15 

fprintf(1, ’hfir_coeffs_q15 = {\n’); 

for i=1:length(hq); 

fprintf(1, ’%5d’, hq(i)); 

804 

20. Using the Arm CMSIS-DSP library 



<!-- Start of picture text -->
Impulse response of filter<br>0.02<br>0.015<br>0.01<br>0.005<br>0<br>–0.005<br>–0.01<br>–0.015<br>–0.02<br>0 20 40 60 80 100 120 140 160 180 200<br><!-- End of picture text -->

FIG. 20.3 Impulse response of the FIR filter. The filter has a strong sine wave component at 697Hz, which is the center of the passband. 



end 

The CMSIS-DSP code to use to apply the FIR filter is straightforward. First, the function arm_fir_init_q15() is called to initialize the FIR filter structure. The function just checks to make sure that the filter length is even and greater than four samples, and then sets a few structural elements. Next, the function processes the signal one block at a time. A macro BLOCKSIZE is defined as 32 and equals the number of samples that will be processed in each call to the arm_fir_q15() function. Each call generates 32 new output samples, and these are written into the output buffer. 

The output of the filter was computed for 697Hz and 770Hz inputs and the results are shown in Fig. 20.5. The top plot shows the output when the input is 697Hz. The sine wave falls into the middle of the band and has the expected output amplitude of 0.5 (Remember that the input sine wave has amplitude of 0.5 and the filter has a gain of 1.0 in the center 

805 

20.4 Example 1—DTMF demodulation 



<!-- Start of picture text -->
Frequency response of FIR filter<br>1.2<br>1<br>0.8<br>0.6<br>0.4<br>0.2<br>0<br>0 500 1000 1500 2000 2500 3000 3500 4000<br>Passband Detail<br>1<br>0.8<br>0.6<br>0.4<br>0.2<br>0<br>400 500 600 700 800 900 1000<br>Frequency (Hz)<br>Magnitude<br>Magnitude<br><!-- End of picture text -->

FIG. 20.4 Frequency response of the filter. The top figure shows the magnitude response across the entire frequency band. The bottom figure shows the detail around the passband frequency of 697Hz. 

of the band.). The bottom plot shows the output when the input frequency is increased to 770Hz. The output is, as expected, greatly attenuated. 

### 20.4.3 Decoding using an FFT 

The next approach to decoding the DTMF tones that I will explore is by using an FFT. The advantage of using the FFT is that it provides a complete frequency representation of the signal and can be used to simultaneously decode all seven DTMF sine waves. A Q31 FFT and a 

806 

20. Using the Arm CMSIS-DSP library 



<!-- Start of picture text -->
FIR filter output when the input is 697 Hz<br>1<br>0.5<br>0<br>–0.5<br>–1<br>0 100 200 300 400 500 600<br>FIR filter output when the input is 770 Hz<br>1<br>0.5<br>0<br>–0.5<br>–1<br>0 100 200 300 400 500 600<br><!-- End of picture text -->

FIG. 20.5 Output of the FIR filter for two different sine wave inputs. The top plot shows the output when the input is a 697Hz sine wave, which is centered in the middle of the band. The bottom plot shows the output when the input is a 770Hz sine wave and is properly attenuated. 

buffer with a 512 sample length will be used in this example. Since the input data are real, a real transformation (i.e., a real-FFT) will be used. 

The FFT operates on an entire buffer of 512 samples. Several steps are involved. First, the data are windowed in order to reduce the transients at the edge of the buffer. There are several different types of window functions (i.e., methods to extract signal samples within a specific period from a longer sequence of samples): Hamming, Hanning, Blackman, etc.—with the choice depending upon the desired frequency resolution and the separation between the neighboring frequencies. In our application, I used a Hanning, or raised cosine, window. The input signal is shown at the top of Fig. 20.6, and the result after windowing is shown at the bottom. You can see that the windowed version decays smoothly to zero at the edges. All data are represented by Q31 values. 

The Arm CMSIS-DSP library function arm_rfft_init_q31 is applied to the data. The function produces complex frequency domain data. The function arm_cmplx_mag_q31 then computes the magnitude of each frequency bin. The resulting magnitude is shown in Fig. 20.7. Since the FFT is 512 points long and the sample rate is 8000Hz, the frequency spacing per FFT bin is 



The largest magnitude is found at bin 45, which corresponds to a frequency of 703Hz—the closest bin to 697Hz. 

807 

20.4 Example 1—DTMF demodulation 



<!-- Start of picture text -->
Input signal<br>0.6<br>0.4<br>0.2<br>0<br>–0.2<br>–0.4<br>0 10 20 30 40 50 60<br>Time (ms)<br>Windowed signal<br>0.5<br>0<br>–0.5<br>0 10 20 30 40 50 60<br>Time (ms)<br>Amplitude<br>Amplitude<br><!-- End of picture text -->

FIG. 20.6 The top figure shows the input sine wave of 697Hz. Some discontinuities at the signal edges can be seen and these could cause the peak frequency to be misidentified. The bottom figure shows the sine wave after it was windowed by a Hanning window. 

If the input frequency is 770Hz, the FFT produces the magnitude plot shown in Fig. 20.8. The peak occurs at bin 49, which corresponds to 766Hz. 

### 20.4.4 Decoding using a Biquad filter 

The final approach taken was to use a second-order Infinite Impulse Response (IIR) filter to detect the tone. The approach is similar to the Goertzel algorithm that is used in most DSPbased decoders. The Biquad filter is designed to have a pole near the unit circle at the desired frequency of 697Hz and zeros at DC (i.e., 0Hz) and Nyquist (i.e., half of the sampling rate). This yields a narrow bandpass shape. The gain of the filter is adjusted so that it has a peak gain of 1.0 in the passband. By moving the pole closer to the unit circle the sharpness of the filter can be adjusted. I settled on placing the pole at a radius of 0.99 and at an angle of 



The pole forms part of a complex conjugate pair, and there is a matching pole at the negative frequency. The MATLAB code to generate the filter coefficients is shown below. The scaling by K creates a peak gain of 1.0 in the passband. 



<!-- Start of picture text -->
Frquency spectrum<br> Peak at 703 Hz<br>0.1<br>0.05<br>0<br>0 500 1000 1500 2000 2500 3000 3500 4000<br>Frequency (Hz)<br>Magnitude of the FFT output. The peak frequency component occurs at 703Hz, which is the bin closest<br>the actual frequency of 697Hz.<br>Frquency spectrum<br> Peak at 766 Hz<br>0.1<br>0.05<br>0<br>0 500 1000 1500 2000 2500 3000 3500 4000<br>Frequency (Hz)<br>Magnitude<br>Magnitude<br><!-- End of picture text -->

FIG. 20.7 Magnitude of the FFT output. The peak frequency component occurs at 703Hz, which is the bin closest to the actual frequency of 697Hz. 

FIG. 20.8 FFT output when the input frequency is 770. 

809 

20.4 Example 1—DTMF demodulation 

r = 0.99; 

p1 = r * exp(sqrt(-1)*2*pi*FC/SR); % Pole location p2 = conj(p1); % Conjugate pole P = [p1; p2]; % Make the array of poles Z = [1; -1]; % Zeros at DC and Nyquist K = 1 - r; % Gain factor for unity gain SOS = zp2sos(Z, P, K); % Convert to biquad coeffs 

The resulting frequency response of the filter is shown in Fig. 20.9. The filter is sharp and passes frequencies within a very narrow band. 

The CMSIS-DSP library provides two versions of the floating-point Biquad filter: Direct Form I and Transposed Direct Form II. For biquad processing using the floating-point format, the best version to use is always the Transposed Direct Form II. This is because it only requires two state variables per Biquad rather than four. With fixed-point, you should always use Direct Form I, but, in this case, the Transposed Direct Form II is the correct version to use. 

The code for processing the Biquad filter is straightforward. This example uses a secondorder filter, which corresponds to a single Biquad filter stage. The filter has two associated arrays: 

Coefficients—five values 

State variables—two values 



<!-- Start of picture text -->
Frequency response of the Biquad filter<br>1<br>0.5<br>0<br>0 500 1000 1500 2000 2500 3000 3500 4000<br>Passband Detail<br>1<br>0.5<br>0<br>400 500 600 700 800 900 1000<br>Frequency (Hz)<br>Magnitude<br>Magnitude<br><!-- End of picture text -->

FIG. 20.9 Frequency response of the IIR filter used in DTMF tone detection. 

810 

20. Using the Arm CMSIS-DSP library 



<!-- Start of picture text -->
FIR filter output when the input is 697 Hz<br>1<br>0.5<br>0<br>–0.5<br>–1<br>0 100 200 300 400 500 600<br>FIR filter output when the input is 770 Hz<br>1<br>0.5<br>0<br>–0.5<br>–1<br>0 100 200 300 400 500 600<br>Sample<br><!-- End of picture text -->

FIG. 20.10 Output of the Biquad DTMF detection filter. 

These arrays are defined at the top of the function and the coefficient array is set to the values computed by MATLAB. The only change is that the feedback coefficients are negated compared to the standard MATLAB representation. Next, the function arm_biquad_cascade_df2T_init_f32() is called to initialize the Biquad instance structure. The processing of the filter is handled by the arm_biquad_cascade_df2T_f32() function, which is called to process the input data in blocks in a program loop. Each call processes 32 samples (i.e., BLOCKSIZE) through the filter and stores the result in the output array. 

The output of the Biquad filter is shown in Fig. 20.10. The top part of the figure shows the output when the input is at 697Hz; the bottom part shows the output when the input is 770Hz. When the input is 697Hz, the output builds-up to the expected amplitude of 0.5. When the input is a 770Hz sine wave, there is still a bit of signal in the output, and the rejection is not as great as the FIR result shown in Fig. 20.5. Still, the filter does a reasonable job discerning the various signal components. 

### 20.4.5 Example of DTMF code 

The following code demonstrates the three DTMF decoding techniques using the CMSIS-DSP library functions. 

811 

20.4 Example 1—DTMF demodulation 

Example DTMF program code 

#include "IOTKit_CM33_FP.h" #include <stdio.h> #include "arm_math.h" #define L 512 #define SR 8000 #define FREQ 697 // #define FREQ 770 #define BLOCKSIZE 8 q15_t inSignalQ15[L]; q31_t inSignalQ31[L]; float inSignalF32[L]; q15_t outSignalQ15[L]; float outSignalF32[L]; 

q31_t fftSignalQ31[2*L]; q31_t fftMagnitudeQ31[2]; 

#define NUM_FIR_TAPS 202 q15_t hfir_coeffs_q15[NUM_FIR_TAPS] = { 

|-9<br>,|-29,|-40,|-40,|-28,|-7,|17,|38,|
|---|---|---|---|---|---|---|---|
|49<br>,|47,|29,|1,|-30,|-55,|-66,|-58,|
|-31 ,|9,|51,|82,|91,|72,|29,|-28,|
|-82 ,|-117,|-119,|-84,|-20,|57,|124,|160,|
|149 ,|91,|0,|-99,|-176,|-206,|-175,|-88,|
|33<br>,|153,|235,|252,|193,|72,|-80,|-217,|
|-297,|-293,|-199,|-40,|141,|289,|358,|323,|
|189 ,|-9,|-213,|-364,|-412,|-339,|-161,|73,|
|294 ,|436,|453,|336,|114,|-149,|-376,|-499,|
|-477,|-312,|-51,|233,|456,|548,|480,|269,|
|-27 ,|-320,|-525,|-579,|-462,|-207,|113,|404,|
|580 ,|587,|422,|131,|-201,|-477,|-614,|-572,|
|-362,|-45,|287,|534,|626,|534,|287,|-45,|
|-362,|-572,|-614,|-477,|-201,|131,|422,|587,|
|580 ,|404,|113,|-207,|-462,|-579,|-525,|-320,|
|-27 ,|269,|480,|548,|456,|233,|-51,|-312,|
|-477,|-499,|-376,|-149,|114,|336,|453,|436,|
|294 ,|73,|-161,|-339,|-412,|-364,|-213,|-9,|
|189 ,|323,|358,|289,|141,|-40,|-199,|-293,|
|-297,|-217,|-80,|72,|193,|252,|235,|153,|



Continued 

812 

20. Using the Arm CMSIS-DSP library 

33 , -88, -175, -206, -176, -99, 0, 91, 149 , 160, 124, 57, -20, -84, -119, -117, -82 , -28, 29, 72, 91, 82, 51, 9, -31 , -58, -66, -55, -30, 1, 29, 47, 49 , 38, 17, -7, -28, -40, -40, -29, -9 , 0}; 

q31_t hanning_window_q31[L]; 

q15_t hfir_state_q15[NUM_FIR_TAPS + BLOCKSIZE] = {0}; 

float biquad_coeffs_f32[5] = {0.01f, 0.0f, -0.01f, 1.690660431255413f, -0.9801f}; float biquad_state_f32[2] = {0}; 

/*—————————————————————————————————————————————————————————————————————————— main program *-—————————————————————————————————————————————————————————————————————————*/ int main (void) { /* execution starts here */ int i, samp; arm_fir_instance_q15 DTMF_FIR; arm_rfft_instance_q31 DTMF_RFFT; arm_biquad_cascade_df2T_instance_f32 DTMF_BIQUAD; 

// Generates the input sine wave 

// The signal will have an amplitude of 0.5 and a frequency of FREQ Hz // Creates floating-point, Q31, and Q7 versions. for(i=0; i<L; i++) { inSignalF32[i] = 0.5f * sinf(2.0f * PI * FREQ * i / SR); inSignalQ15[i] = (q15_t) (32768.0f * inSignalF32[i]); inSignalQ31[i] = (q31_t) ( 2147483647.0f * inSignalF32[i]); } 

/* ———————————————————————————————————————————————————————————————————— ** Process with FIR filter ** ————————————————————————————————————————————————————————————————— */ 

if (arm_fir_init_q15(&DTMF_FIR, NUM_FIR_TAPS, &hfir_coeffs_q15[0], &hfir_state_q15[0], BLOCKSIZE) != ARM_MATH_SUCCESS) { // error condition // exit(1); } 

for(samp = 0; samp < L; samp += BLOCKSIZE) { 

arm_fir_q15(&DTMF_FIR, inSignalQ15 + samp, outSignalQ15 + samp, BLOCKSIZE); } 

813 

20.5 Example 2—Least squares motion tracking 

/* ———————————————————————————————————————————————————————————————————— ** Process with a floating-point Biquad filter ** ————————————————————————————————————————————————————————————————— */ 

arm_biquad_cascade_df2T_init_f32(&DTMF_BIQUAD, 1, biquad_coeffs_f32, 

biquad_state_f32); 

for(samp = 0; samp < L; samp += BLOCKSIZE) { 

arm_biquad_cascade_df2T_f32(&DTMF_BIQUAD, inSignalF32 + samp, 

outSignalF32 + samp, BLOCKSIZE); 

} 

/* ———————————————————————————————————————————————————————————————————— ** Process with Q31 FFT ** ————————————————————————————————————————————————————————————————— */ // Creates the Hanning window. This is usually done once at the // start of the program. 

for(i=0; i<L; i++) { hanning_window_q31[i] = (q31_t) (0.5f * 2147483647.0f * (1.0f - cosf(2.0f*PI*i / L))); } // Applies the window to the input buffer arm_mult_q31(hanning_window_q31, inSignalQ31, inSignalQ31, L); 

arm_rfft_init_q31(&DTMF_RFFT, 512, 0, 1); 

// Computes the FFT arm_rfft_q31(&DTMF_RFFT, inSignalQ31, fftSignalQ31); 

arm_cmplx_mag_q31(fftSignalQ31, fftMagnitudeQ31, L); 

} 

### 20.5 Example 2—Least squares motion tracking 

Tracking the motion of an object is a common requirement for many applications. Examples include navigation systems, exercise equipment, video game controllers, and factory automation. Noisy measurements of past positions of the object are available and they need to be combined to estimate future positions. One approach to solve this challenge is to combine multiple noisy measurements in order to estimate the underlying trajectory (position, velocity, and acceleration) and to then project this into the future. 

814 

20. Using the Arm CMSIS-DSP library 

Consider an object under constant acceleration. The motion of the object as a function of time t will be: 



where 

x0 is the initial position 

v0 is the initial velocity 

a is the acceleration 

In this example, I will assume that the acceleration is constant but the approach used can be extended to time-varying acceleration. 

Assume that we have measurements of the past location of the object at times t1, t2, …, tN. Assume also that the position measurements are noisy and that we only know an approximate position. Place the measurements and times into column vectors: 



The overall equation that relates the measurements to the unknowns x0, v0, and a is then 



This expression can be computed using a matrix multiplication 

x ¼ Ac 

where 



and 



Since there are three unknowns, at least three measurements are required to compute the result vector c. In most cases, there are many more measurements than unknowns, and the problem is overdetermined. One standard solution to this problem is to carry out the least-squares fit. The solution ^c is the one which minimizes the error between 

815 

20.5 Example 2—Least squares motion tracking 

the N estimated positions and the actual N measurement positions. The least-squares solution can be found by solving the matrix equation: 



Equations of this type can be solved using the matrix functions within the CMSIS-DSP library. A matrix in the CMSIS-DSP library is represented using a data structure. For floating-point data, the structure is: 

typedef struct { uint16_t numRows; /**< number of rows of the matrix. */ uint16_t numCols; /**< number of columns of the matrix. */ float32_t *pData; /**< points to the data of the matrix. */ } arm_matrix_instance_f32; 

Essentially, the matrix structure keeps track of the size of the matrix (numRows, numCols) and contains a pointer to the data (pData). Element (R, C) of the matrix is stored at location 

pData[R*numRows + C] 

in the array. That is, the array contains the first row of data, followed by the second row of data, and so on. You can either manually initialize the matrix instance structure yourself by setting internal fields or you can use the function arm_mat_init_f32(). In practice, it is easier to manually initialize the matrix. 

The overall code that computes the least-squares solution is shown in Section 20.5.1. The top of the function allocates memory for all of the pData arrays used by the matrices. Matrices t and x are initialized with actual data while all other matrices are initially set to zero. After this, the individual matrix instance structures are initialized. Note: multiple matrices are defined so that the intermediate results can be stored. The following matrices are defined: 

A—matrix A from above AT—transpose of A ATA—the product A<sup>T</sup> A invATA—the inverse of A<sup>T</sup> A B—the product (A<sup>T</sup> A)<sup>�1</sup> A<sup>T</sup> c—the final result from above 

The start of the main function initializes the values for matrix A. After this, several matrix math functions are called to finally arrive at the result vector c. The result contains three elements and they can be seen by inspecting the cData within the debugger. By so doing, it will be found that: 



The original input data and the resulting fit are shown in Fig. 20.11. The thin line is the measured data showing the random noise, and the thick line is the resulting data fit. The figure shows that the underlying least-squares fit is quite accurate, and the fit could be used to extrapolate the measured data into the future. 

816 

20. Using the Arm CMSIS-DSP library 



<!-- Start of picture text -->
Measured data<br>60<br>Measurements<br>Fit<br>40<br>20<br>0<br>–20<br>–40<br>–60<br>0 0.5 1 1.5 2 2.5 3 3.5 4 4.5 5<br>Time<br>Meters<br><!-- End of picture text -->

FIG. 20.11 Raw measurements (thin line) and resulting least-squares fit (thick line). 

### 20.5.1 Example of least squares code 

Example code for least square computation 

#include "arm_math.h" /* Main include file for CMSIS-DSP */ #define NUMSAMPLES 51 /* Number of measurements */ #define NUMUNKNOWNS 3 /* Number of unknowns in polynomial fit */ 

// Allocate memory for the matrix arrays. Only t and x have initial data defined. 

// Contains the times at which the data was sampled. In this example, the data // is evenly spaced but this is not required for the least squares fit. float32_t tData[NUMSAMPLES] = { 0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f, 1.1f, 1.2f, 1.3f, 1.4f, 1.5f, 1.6f, 1.7f, 1.8f, 1.9f, 2.0f, 2.1f, 2.2f, 2.3f, 2.4f, 2.5f, 2.6f, 2.7f, 2.8f, 2.9f, 3.0f, 3.1f, 3.2f, 3.3f, 3.4f, 3.5f, 3.6f, 3.7f, 3.8f, 3.9f, 4.0f, 4.1f, 4.2f, 4.3f, 4.4f, 4.5f, 4.6f, 4.7f, 4.8f, 4.9f, 5.0f }; 

817 

20.5 Example 2—Least squares motion tracking 

// Contains the noisy position measurements float32_t xData[NUMSAMPLES] = { 7.4213f, 21.7231f, -7.2828f, 21.2254f, 20.2221f, 10.3585f, 20.3033f, 29.2690f, 57.7152f, 53.6075f, 22.8209f, 59.8714f, 43.1712f, 38.4436f, 46.0499f, 39.8803f, 41.5188f, 55.2256f, 55.1803f, 55.6495f, 49.8920f, 34.8721f, 50.0859f, 57.0099f, 47.3032f, 50.8975f, 47.4671f, 38.0605f, 41.4790f, 31.2737f, 42.9272f, 24.6954f, 23.1770f, 22.9120f, 3.2977f, 35.6270f, 23.7935f, 12.0286f, 25.7104f, -2.4601f, 6.7021f, 1.6804f, 2.0617f, -2.2891f, -16.2070f, -14.2204f, -20.1870f, -18.9303f, -20.4859f, -25.8338f, -47.2892f 

}; 

float32_t AData[NUMSAMPLES * NUMUNKNOWNS]; float32_t ATData[NUMSAMPLES *NUMUNKNOWNS]; float32_t ATAData[NUMUNKNOWNS * NUMUNKNOWNS]; float32_t invATAData[NUMUNKNOWNS * NUMUNKNOWNS]; float32_t BData[NUMUNKNOWNS * NUMSAMPLES]; float32_t cData[NUMUNKNOWNS]; // Array instance structure initialization. For each instance, the form is: // MAT = {numRows, numCols, pData}; 

// Column vector t arm_matrix_instance_f32 t = {NUMSAMPLES, 1, tData}; 

// Column vector x arm_matrix_instance_f32 x = {NUMSAMPLES, 1, xData}; 

// Matrix A arm_matrix_instance_f32 A = {NUMSAMPLES, NUMUNKNOWNS, AData}; 

// Transpose of matrix A arm_matrix_instance_f32 AT = {NUMUNKNOWNS, NUMSAMPLES, ATData}; 

// Matrix product AT * A 

arm_matrix_instance_f32 ATA = {NUMUNKNOWNS, NUMUNKNOWNS, ATAData}; 

// Matrix inverse inv(AT*A) 

arm_matrix_instance_f32 invATA = {NUMUNKNOWNS, NUMUNKNOWNS, invATAData}; 

// Intermediate result invATA * AT 

arm_matrix_instance_f32 B = {NUMUNKNOWNS, NUMSAMPLES, BData}; 

Continued 

818 

20. Using the Arm CMSIS-DSP library 

// Solution arm_matrix_instance_f32 c = {NUMUNKNOWNS, 1, cData}; /*—————————————————————————————————————————————————————————————————————————— ** main program **————————————————————————————————————————————————————————————————————————*/ int main (void) { int i; float y; y = sqrtf(xData[0]); cData[0] = y; // Fills in the values for matrix A. Each row contains: // [1.0f t t*t] for(i=0; i<NUMSAMPLES; i++) { AData[i*NUMUNKNOWNS + 0] = 1.0f; AData[i*NUMUNKNOWNS + 1] = tData[i]; AData[i*NUMUNKNOWNS + 2] = tData[i] * tData[i]; } // Transpose arm_mat_trans_f32(&A, &AT); // Matrix multplication AT * A arm_mat_mult_f32(&AT, &A, &ATA); // Matrix inverse inv(ATA) arm_mat_inverse_f32(&ATA, &invATA); // Matrix multiplication invATA * x; arm_mat_mult_f32(&invATA, &AT, &B); // Final result. arm_mat_mult_f32(&B, &x, &c); // Examine cData in the debugger to view the final values } 

### 20.6 Example 3—Real-time filter design 

### 20.6.1 Overview of filter design 

Filtering is one of the most commonly used signal processing tasks in real-time embedded systems. In Section 20.4 I showed how to apply filtering to an input signal sequence. However, in many applications the input signals are active all of the time and, thus, continuous filtering is needed. In this section, I am going to look into how to create real-time filters using the CMSIS-DSP library. 

819 

20.6 Example 3—Real-time filter design 

The first step of filter design is to define the filter specification. Some of the areas that need to be defined include: 

- Filter type 

- Data type used by the digital filter 

- Sampling frequency 

- Frequency response 

The frequency response of a filter can be characterized using various aspects. For example, Fig. 20.12 shows some of these characteristics. 

There can be a lot of factors to consider when designing filters. In addition to the frequency response (e.g., rejection ratio), phase response can also be important in some applications. In some cases, you might also need to take the delay of the filter response into account when making the design decision. Although filter design techniques are not covered in detail in this book, information about filter design is widely available (e.g., an IIR filter design guide from Advanced Solution Nederland provides a good overview of this topic [2]). There is also a range of filter design tools to make life easier. 



<!-- Start of picture text -->
Rp (Passband ripple in dB)<br>0<br>Transition band (lower and upper<br>–20<br>cut-off frequency).<br>–40<br>–60 Rs (Stopband attenuation in dB)<br>–80<br>–100<br>–120<br>|<br>–140<br>–160<br>0 25 50 75 100 125 150 175 200 225<br>Frequency (Hz)<br>Log magnitude (dB)<br><!-- End of picture text -->

FIG. 20.12 A range of filter characteristics to consider in filter design. Image courtesy of Advanced Solution Nederland. 

820 

20. Using the Arm CMSIS-DSP library 

In Section 20.4, the use of MATLAB for filter design was briefly covered. MATLAB is one of the most popular mathematical tools available on the market and includes features that support the design of filters. There are other filter-design software tools available on the market, including some in the form of open-source projects. 

A number of commercial filter-design software tools are designed specifically for filterdesign tasks and make it easier to analyze a filter’s characteristics. For software developers who are not familiar with filter designs, these tools can be a great help. In the following examples, the filters were created using ASN Filter Designer Professional (Version 4.33), which have been developed by Advanced Solution Nederland (http://www.advsolned.com). 

The ASN Filter Designer Professional software supports a wide range of filter types. Its design allows filters to be designed via an interactive user interface, where various parameters can be adjusted and the design’s output can immediately be viewed. It also supports the simulation of the filter’s response so that the simulation outputs can be examined to determine whether the filter meets the requirements of the application. An added bonus, for developers creating software for Cortex-M processors, is that it generates C codes that directly call CMSIS-DSP library functions (the designed filters can also be exported to C/C++, Python, MATLAB, etc.). 

When the ASN Filter Designer starts, and assuming that the license key has been installed, a default startup screen, as shown in Fig. 20.13, will appear. 

In this example, I am going to design a low pass biquad filter for a system with 48kHz sampling rate and with single-precision floating-point data types. The first step to take when starting a basic filter design is to define the sampling rate. This is achieved by clicking on the “Fs” icon on the toolbar and entering the sampling rate in the Sampling Frequency dialog (Fig. 20.14). 



<!-- Start of picture text -->
Beebe | RRs CH |e | S pmagencenmae Fe] He | AD aby Oe<br>— Leg magatese — Pane — Onign onctenton m__t2__0_| Mom<br>Mares [Bie <) Beeuse<br>of c Secione2<br>Tw (Lowpas | ties<br>2 0 ower [& | Base<br>al “y Sond 2sFregeFa)15<br>‘cre, 120 5<br>{ 10 F<br>0 ‘ 15<br>g 200 ®<br>200 RG %<br>ue = 85<br>100. © 2 60 75 100pennies,125 150 175 200 235 250 6 1s 1S as fate0s tS<br>poe<br><!-- End of picture text -->

FIG. 20.13 Default startup screen for the ASN Filter Designer Professional. 

821 

20.6 Example 3—Real-time filter design 



<!-- Start of picture text -->
e ~[Fex| He | AB | @By| @-<br>Sampling Frequency Q Notes<br>40, is |<br>mae<br>(2001-10000 ees<br>cea<br>40 pee tic ie =<br>Apply | =<br>a 2 AttRippe (68)<br><!-- End of picture text -->

FIG. 20.14 Setting up the sampling rate. 

The next step is to define the filter type. By default, the filter type is “Elliptic IIR” filter; this can be changed to a simple Butterworth Biquad filter by selecting the filter “Method” on the top right-hand side of the user interface (Fig. 20.15). 

The next step is to then define the data type and structure. These settings can be found by clicking on the “Q” tab on the top right-hand side of the user interface, as shown in Fig. 20.16. For Cortex-M33 processors with an FPU, because the FPU supports a single-precision floating-point unit this is the option I have selected for this example. A fixed-point data format can, if preferred, also be selected. If that is required, a suitable fixed-point data format (Word length and number of bits for the fractional part in the data) should be selected. 

At this stage of the process, the filter characteristic (Fig. 20.17) should be defined. To do so, the IIR tab should be clicked again, whereupon the number of biquad sections required, and the filter order will be displayed. The square cursors in the graph on the left-hand side can be adjusted to control the filter’s frequency response. After you have adjusted the filter response, you then need to click on the “Design” button in the IIR filter tab (on the right) to update the filter design. 



<!-- Start of picture text -->
=a<br>| He | | <br]1 @-<br>WR FIR) PZ Qs<br>Method GA Biquads<br>Butterworth Sections: 3<br>Type [Chebyshev<br>Chebyshev Type | .<br>Elliptic Type Il SaaS<br>Order lUser Defined _ea<br>Band Frequencies (kHz) ‘Att/Ripple (48)<br>1 0,24 15<br>2 12,24 80<br><!-- End of picture text -->

FIG. 20.15 Defining the filter type. 

822 

20. Using the Arm CMSIS-DSP library 

FIG. 20.16 Defining the filter’s data type and structure. 



<!-- Start of picture text -->
“| He | 93 | «By| @- ininin<br>UR FIR P-Z Q Notes<br>Arithmetic Single Precision i scaling<br>Structure Direct Form Il Transposed ~ 2<br>FWL options<br>16<br>is [2] 8<br>Apply =<br><!-- End of picture text -->



<!-- Start of picture text -->
BER [ RA eH [| leomagnine rm Fee | He | AL eBy| @<br>— Log magnitude Phase —= Design specification id fa Fx. S| ees.<br>0- Method Butterworth, a © Biauads:<br>20. NY - ‘Stable Yes<br>\ 180 orser [8] Auto<br>a0 \ os and Frequences3) Ape3)<br>\<br>Ae ~, ||P a<br>: X i 5<br>9 \ “ ‘<br>LY<br>\ os<br>160. Y + + + 730 "<br>ae is esas Ts<br>ais<br>er SS<br>|<br><!-- End of picture text -->

FIG. 20.17 Defining the frequency response of the filter. 

Once the filter is defined, the design can then be exported into C code (using the Arm CMSIS-DSP option). The C code contains the generated filter coefficients and the function calls that access the required CMSIS-DSP library functions. The function to export the C code is accessed by clicking on the “H(z)” button on the toolbar (Fig. 20.18). 

20.6 Example 3—Real-time filter design 



<!-- Start of picture text -->
823<br><!-- End of picture text -->



<!-- Start of picture text -->
a<br>Brd~ be | RAx+ O+| 4 | WF Logmagnitude& Phase Fe| He | 99 | @Br| @~<br>— eg magnitude — Phase — Design specication 0 UR FR PZ OQ_(Notes<br>° Method [Butterworth © Biquads<br>N Sections:4<br>x (d| 0 7 | Am cosis-ose -<br>\ 7] 33H Fiiver Des{Doaimentation Ee) ro<br>ad \\ 7 Sat, 08 Fed 204ythonMatlab/Octave Ripple (68)<br>QC] esse TAs Cormpone F<br>& AG) eanat 4Sclab st/Ripple (8)<br>§ 6. Yider' olcices 0.002<br>3 “2 BEB arm CMSIS-DSP<br>7 17 bevcmecic1 = PARSERes Fm Comper vt] @ OK toFramework depicy tothe Arm CMSIS-OSP<br>“120. 1) Filver Order = 8:<br>140.<br>-160. ° 4 Om &#sefsne NUMSECTIONStn 4 > ’<br>Frequency (kz) a Real) a<br>Box<br><!-- End of picture text -->

FIG. 20.18 Exporting the filter design to C code using the CMSIS-DSP library. 

The generated biquad filter code is as follows: 

Generated Biquad filter C code utilizing the CMSIS-DSP library 

- // ASN Filter Designer Professional v4.3.3 

- // Sat, 08 Feb 2020 11:50:38 GMT 

// ** Primary Filter (H1)** ////Band# Frequencies (kHz) Att/Ripple (dB) // 1 0.000, 2.400 0.001 // 2 12.000, 24.000 80.000 //// 

- // Arithmetic = ’Floating Point (Single Precision)’; 

- // Architecture = ’IIR’; 

- // Structure = ’Direct Form II Transposed’; 

- // Response = ’Lowpass’; 

- // Method = ’Butterworth’; 

- // Biquad = ’Yes’; 

Continued 

#### 824 

20. Using the Arm CMSIS-DSP library 

// Stable = ’Yes’; // Fs = 48.0000; //kHz // Filter Order = 8; // ** ASN Filter Designer Automatic Code Generator ** // ** Deployment to Arm CMSIS-DSP Framework ** #include "arm_math.h" #define NUM_SECTIONS_IIR 4 // ** IIR Direct Form II Transposed Biquad Implementation ** // y[n] = b0 * x[n] + w1 // w1 = b1 * x[n] + a1 * y[n] + w2 // w2 = b2 * x[n] + a2 * y[n] // IIR Coefficients float32_t iirCoeffsf32[NUM_SECTIONS_IIR*5] = {// b0, b1, b2, a1, a2 0.0582619, 0.1165237, 0.0582619, 1.0463270, -0.2788444, 0.0615639, 0.1231277, 0.0615639, 1.1071010, -0.3531238, 0.0688354, 0.1376707, 0.0688354, 1.2402050, -0.5158056, 0.0815540, 0.1631081, 0.0815540, 1.4713260, -0.7982877 }; 

- // ********************************************************** // Test loop code (Right mouse button -> Show test loop code) 

// ********************************************************** #define ARM_MATH_CM4 // Cortex-M4 (default). Can be omitted if deploying to CMSIS-DSP v1.7.0 or greater 

#define TEST_LENGTH_SAMPLES 128 #define BLOCKSIZE 32 #define NUMBLOCKS (TEST_LENGTH_SAMPLES/BLOCKSIZE) float32_t iirStatesf32[NUM_SECTIONS_IIR*5]; float32_t OutputValues[TEST_LENGTH_SAMPLES]; float32_t InputValues[TEST_LENGTH_SAMPLES]; float32_t *InputValuesf32_ptr = &InputValues[0]; // declares Input pointer float32_t *OutputValuesf32_ptr = &OutputValues[0]; // declares Output pointer arm_biquad_cascade_df2T_instance_f32 S; int32_t main(void) { uint32_t n,k; 

825 

20.6 Example 3—Real-time filter design 

// sets up the test sinusoid input 

for (n=0; n<TEST_LENGTH_SAMPLES; n++) 

InputValues[n]= arm_sin_f32(2*PI*4.8*n/48.0); 

// Initialises the Biquads 

arm_biquad_cascade_df2T_init_f32 (&S, NUM_SECTIONS_IIR, &(iirCoeffsf32[0]), & (iirStatesf32[0])); 

// Performs the IIR filtering operation 

for (k=0; k < NUMBLOCKS; k++) 

arm_biquad_cascade_df2T_f32 (&S, InputValuesf32_ptr + (k*BLOCKSIZE), OutputValuesf32_ptr + (k*BLOCKSIZE), BLOCKSIZE); // performs filtering 

while (1); } 

If the selected filter design is not supported by the CMSIS-DSP library, the filter design can be exported as C/C++ codes instead. 

By default, the generated C code assumes that the Cortex-M4 processor is being used. Because the CMSIS-DSP APIs for all Cortex-M processors are identical, the code can be used on the Cortex-M33 processor. By using a set of precalculated input values (which are stored in a data array), the generated code demonstrates the operation of the filter design. The program ends when all the data in the input data array has been processed. To convert the code into a real-time filter, which processes input data continuously, the code would need to be modified. 

### 20.6.2 Creating a real-time filter: A bare metal project example 

To make the filter computation more efficient, the filter in the CMSIS-DSP library only executes after a block of samples has been collected. The block size is dependent on the filter design and, in the example biquad I have highlighted, the block size is 32—which means the filter function executes every time 32 samples have been collected. To support this behavior, the input and output buffers need to be defined (Fig. 20.19). 

If the filter processing function finishes within 1 sampling period, then the same set of input and output buffers can, after the biquad filter calculation has taken place, be immediately reused for the next block of samples. However, the processing time needed for the filter function to execute often exceeds the timing gap of one sample. As a result, having just one set of input and output buffers is not enough. To solve this problem, two sets of input and output data buffers are, typically, needed. When one pair of input and output buffer’s is used for the filter processing, the other pair is used for the input of data samples and the output of the previously executed processing results. This is often called a ping-pong buffer operation (Fig. 20.20). 

826 

20. Using the Arm CMSIS-DSP library 





FIG. 20.19 CMSIS-DSP filter functions operate on blocks of samples and therefore require input and output buffers. 

For the purpose of demonstrating a real-time filter using a “ping-pong” buffer operation, I have created an example code for an audio filter. In this example, the input and the output are handled by an audio interface interrupt handler. This triggers the sampling rate (i.e., 48kHz in my setup) and the biquad filter function runs in thread mode (which can be interrupted by the audio interface interrupt). To enable the processing to be kick-started after a buffer is filled, the following data variables are required: 

- A data variable for counting the number of samples collected in the input buffer A/B (“Sample_Counter”) 

- A data variable to indicate ping-pong state (“PingPongState”) 

- A data variable, which allows the audio interrupt handler to trigger the start of the filter processing once enough samples have been collected (“processing_trigger”) 

- And of course, two pairs of input/output buffers (“InputValues_A/B”, “OutputValues_A/B”) 

At the start of the program, the buffers and the aforementioned variables, the biquad filter data structure, and the audio interface hardware (including the interrupt configuration of the NVIC) will all need to be initialized. Because the filter code generated by the ASN Filter Designer targets the Cortex-M4 processor by default, the code contains the C macro “ARM_MATH_CM4”. This C macro is not used in the recent releases of the CMSIS-DSP library and can be ignored. 



<!-- Start of picture text -->
20.6 Example 3—Real-time filter design 827<br><!-- End of picture text -->



FIG. 20.20 CMSIS-DSP filter with a “ping pong” buffer. 

828 

20. Using the Arm CMSIS-DSP library 

After all the aforementioned modifications have been carried out, the real-time audio filter design can then be tested. The program code for doing this is detailed below: 

Real-time filter code // ASN Filter Designer Professional v4.3.3 // Fri, 07 Feb 2020 18:41:12 GMT // ** Primary Filter (H1)** ////Band# Frequencies (kHz) Att/Ripple (dB) // 1 0.000, 2.400 0.001 // 2 12.000, 24.000 80.000 //// // Arithmetic = ’Floating Point (Single Precision)’; // Architecture = ’IIR’; // Structure = ’Direct Form II Transposed’; // Response = ’Lowpass’; // Method = ’Butterworth’; // Biquad = ’Yes’; // Stable = ’Yes’; // Fs = 48.0000; //kHz // Filter Order = 8; // ** ASN Filter Designer Automatic Code Generator ** // ** Deployment to Arm CMSIS-DSP Framework ** // Included Headers #include "IOTKit_CM33_FP.h" #include "arm_math.h" // CMSIS-DSP library header extern uint8_t audio_init(void); // Initializes audio hardware extern void read_sample(int16_t *left, int16_t *right); extern void play_sample(int16_t *left, int16_t *right); void I2S_Handler(void); #define BLOCKSIZE 32 #define NUM_SECTIONS_IIR 4 // ** IIR Direct Form II Transposed Biquad Implementation ** // y[n] = b0 * x[n] + w1 // w1 = b1 * x[n] + a1 * y[n] + w2 // w2 = b2 * x[n] + a2 * y[n] // IIR Coefficients // IIR Coefficients float32_t const static iirCoeffsf32[NUM_SECTIONS_IIR*5] = 

829 

20.6 Example 3—Real-time filter design 

{// b0, b1, b2, a1, a2 0.0582619f, 0.1165237f, 0.0582619f, 1.0463270f, -0.2788444f, 0.0615639f, 0.1231277f, 0.0615639f, 1.1071010f, -0.3531238f, 0.0688354f, 0.1376707f, 0.0688354f, 1.2402050f, -0.5158056f, 0.0815540f, 0.1631081f, 0.0815540f, 1.4713260f, -0.7982877f }; 

float32_t static iirStatesf32[NUM_SECTIONS_IIR*5]; 

static arm_biquad_cascade_df2T_instance_f32 S; // Ping-pong buffer float32_t static InputValues_A[BLOCKSIZE]; float32_t static OutputValues_A[BLOCKSIZE]; float32_t static InputValues_B[BLOCKSIZE]; float32_t static OutputValues_B[BLOCKSIZE]; volatile int static PingPongState=0; // 0 = processing A, I/O = B // 1 = processing B, I/O = A volatile int static Sample_Counter=0; // Counts sample from 0 to BLOCKSIZE-1, // then triggers processing and returns to 0 volatile int static processing_trigger=0; // When it is set to 1, it starts processing // Sets to 2 during processing & returns to 0 when processing is completed // Samples from both audio channels int16_t static left_channel_in; int16_t static right_channel_in; int16_t static left_channel_out; int16_t static right_channel_out; int main(void) { int i; // Initializes data in ping-pong buffer for (i=0;i<BLOCKSIZE;i++) { OutputValues_A[i] = 0.0f; OutputValues_B[i] = 0.0f; } // Initializes Biquads arm_biquad_cascade_df2T_init_f32 (&S, NUM_SECTIONS_IIR, &(iirCoeffsf32[0]), & (iirStatesf32[0])); audio_init(); // Initializes the audio interface 

Continued 

830 

20. Using the Arm CMSIS-DSP library 

// Waits for I2S IRQs to occur and then services them. // WFE sleep is used to allow power saving while(1){ if (processing_trigger>0) { // processing_trigger is set to 1 by I2S_Handler when a BLOCK of data is sampled processing_trigger=2; // indicates the biquad is running if (PingPongState==0) { arm_biquad_cascade_df2T_f32 (&S, &InputValues_A[0], &OutputValues_A[0], BLOCKSIZE); // performs filtering } else { arm_biquad_cascade_df2T_f32 (&S, &InputValues_B[0], &OutputValues_B[0], BLOCKSIZE); // performs filtering } // endif-if (PingPongState==0) processing_trigger=0; // returns state to 0 } // end-if (processing_trigger!=0) __WFE(); // enters sleep when the processing is finished } } /************************************************************************/ /* I2S audio IRQ handler. Triggers at 48KHz. */ /************************************************************************/ void I2S_Handler(void) { int local_Sample_Counter; // Sample_Counter counts from 0 to BLOCKSIZE-1 local_Sample_Counter = Sample_Counter; // Reads sample from the ADC read_sample(&left_channel_in, &right_channel_in); if (PingPongState==0) { InputValues_B[local_Sample_Counter] = (float) left_channel_in; left_channel_out=(int16_t) OutputValues_B[local_Sample_Counter]; } else { InputValues_A[local_Sample_Counter] = (float) left_channel_in; left_channel_out=(int16_t) OutputValues_A[local_Sample_Counter]; } right_channel_out = right_channel_in; // Only left channel is processed // Writes sample to the DAC play_sample(&left_channel_out, &right_channel_out); local_Sample_Counter++; if (local_Sample_Counter>=BLOCKSIZE) { // Wraps around and toggles the ping-pong buffer 

831 

20.6 Example 3—Real-time filter design 

local_Sample_Counter = 0; 

PingPongState = (PingPongState+1) & 0x1; // toggles the ping-pong state if (processing_trigger==2) { 

- // Biquad is still running – an overrun error has occurred 

__BKPT(1); // Error 

} else { processing_trigger = 1; // Starts new biquad } } Sample_Counter = local_Sample_Counter; // saves new Sample_Counter return; } 

In a real-world application, a device is likely to have a number of interrupt sources. In the unlikely scenario that the processor receives a large number of interrupt requests in a short period of time, the filter function might not be able to complete the processing before the next block of samples is received. To detect when this occurs, a check is carried out inside the I2S handler. If the value of the “processing_trigger” is still 2 when a block of samples is received, this is flagged up as an error as it means the filter processing function running in the thread mode is still running. 

### 20.6.3 Filter processing in a low priority handler 

To reduce the possibility of the processor not being able to finish the filter processing in time for the next block of data, the filter processing function could be run as another interrupt handler at a low priority level (i.e., a lower priority than the audio interface interrupt). This arrangement also allows us to run some slow processing functions (nonreal-time) at the thread level. By running the filter processing as an interrupt handler, codes for measuring the filter’s processing duration and for displaying the result (using a printf statement) can be added to the project in the form of thread-level background operations. This arrangement avoids the filter processing from being delayed. 

In the following example, the same filter algorithm that was used in the previous example is used, but the execution of the filter function is carried out inside the PendSV exception handler. 

Real-time filter code using the PendSV exception for filter processing. Thread mode is used for displaying the processor’s load. 

> // ASN Filter Designer Professional v4.3.3 

> // Fri, 07 Feb 2020 18:41:12 GMT 

> // ** Primary Filter (H1)** 

Continued 

832 

20. Using the Arm CMSIS-DSP library 

////Band# Frequencies (kHz) Att/Ripple (dB) // 1 0.000, 2.400 0.001 // 2 12.000, 24.000 80.000 //// // Arithmetic = ’Floating Point (Single Precision)’; // Architecture = ’IIR’; // Structure = ’Direct Form II Transposed’; // Response = ’Lowpass’; // Method = ’Butterworth’; // Biquad = ’Yes’; // Stable = ’Yes’; // Fs = 48.0000; //kHz // Filter Order = 8; // ** ASN Filter Designer Automatic Code Generator ** // ** Deployment to Arm CMSIS-DSP Framework ** // Included Headers #include "IOTKit_CM33_FP.h" #include "arm_math.h" // CMSIS-DSP library header #include "stdio.h" extern uint8_t audio_init(void); // Initializes audio hardware extern void read_sample(int16_t *left, int16_t *right); extern void play_sample(int16_t *left, int16_t *right); void I2S_Handler(void); void PendSV_Handler(void); #define BLOCKSIZE 32 #define NUM_SECTIONS_IIR 4 // ** IIR Direct Form II Transposed Biquad Implementation ** // y[n] = b0 * x[n] + w1 // w1 = b1 * x[n] + a1 * y[n] + w2 // w2 = b2 * x[n] + a2 * y[n] // IIR Coefficients // IIR Coefficients float32_t const static iirCoeffsf32[NUM_SECTIONS_IIR*5] = {// b0, b1, b2, a1, a2 0.0582619f, 0.1165237f, 0.0582619f, 1.0463270f, -0.2788444f, 0.0615639f, 0.1231277f, 0.0615639f, 1.1071010f, -0.3531238f, 0.0688354f, 0.1376707f, 0.0688354f, 1.2402050f, -0.5158056f, 0.0815540f, 0.1631081f, 0.0815540f, 1.4713260f, -0.7982877f }; 

833 

20.6 Example 3—Real-time filter design 

float32_t static iirStatesf32[NUM_SECTIONS_IIR*5]; 

static arm_biquad_cascade_df2T_instance_f32 S; // Ping-pong buffer float32_t static InputValues_A[BLOCKSIZE]; float32_t static OutputValues_A[BLOCKSIZE]; float32_t static InputValues_B[BLOCKSIZE]; float32_t static OutputValues_B[BLOCKSIZE]; volatile int static PingPongState=0; // 0 = processing A, I/O = B 

// 1 = processing B, I/O = A 

volatile int static Sample_Counter=0; // Counts sample from 0 to BLOCKSIZE-1, // then triggers processing and returns to 0 volatile int static processing_trigger=0; // When it is set to 1, it starts processing 

// Sets to 2 during processing & returns to 0 when processing is completed 

// Samples from both audio channels int16_t static left_channel_in; int16_t static right_channel_in; int16_t static left_channel_out; int16_t static right_channel_out; // Number of cycles per block of samples #define CPUCYCLE_MAX ((20000000/48000)*BLOCKSIZE) static volatile uint32_t cycle_cntr=0; int main(void) { int i; // Initializes data in ping-pong buffer for (i=0;i<BLOCKSIZE;i++) { OutputValues_A[i] = 0.0f; OutputValues_B[i] = 0.0f; } // Initializes Biquads arm_biquad_cascade_df2T_init_f32 (&S, NUM_SECTIONS_IIR, &(iirCoeffsf32[0]), & (iirStatesf32[0])); 

NVIC_SetPriority(PendSV_IRQn, 7); // PendSV sets to Low priority 

audio_init(); // Initializes the audio interface // Waits for I2S IRQs to occur and then services them. 

Continued 

834 

20. Using the Arm CMSIS-DSP library 

while(1){ 

printf ("cpu load=%d percent\n", (100*cycle_cntr/CPUCYCLE_MAX)); } } /************************************************************************/ /* PendSV handler. Triggers at 48KHz/BLOCKSIZE. */ /************************************************************************/ void PendSV_Handler(void) { processing_trigger=2; // indicates the biquad is running SysTick->LOAD=0x00FFFFFFUL; SysTick->VAL=0; SysTick->CTRL=SysTick_CTRL_CLKSOURCE_MskjSysTick_CTRL_ENABLE_Msk; if (PingPongState==0) { 

arm_biquad_cascade_df2T_f32 (&S, &InputValues_A[0], &OutputValues_A[0], BLOCKSIZE); // performs filtering } else { arm_biquad_cascade_df2T_f32 (&S, &InputValues_B[0], &OutputValues_B[0], BLOCKSIZE); // performs filtering } // endif-if (PingPongState==0) processing_trigger=0; // returns state to 0 cycle_cntr = 0x00FFFFFFUL - SysTick->VAL; SysTick->CTRL=0; return; } 

- /************************************************************************/ /* I2S audio IRQ handler. Triggers at 48KHz. */ /************************************************************************/ void I2S_Handler(void) { int local_Sample_Counter; 

// Sample_Counter counts from 0 to BLOCKSIZE-1 local_Sample_Counter = Sample_Counter; 

// Reads sample from the ADC 

read_sample(&left_channel_in, &right_channel_in); 

if (PingPongState==0) { 

InputValues_B[local_Sample_Counter] = (float) left_channel_in; 

left_channel_out=(int16_t) OutputValues_B[local_Sample_Counter]; } else { InputValues_A[local_Sample_Counter] = (float) left_channel_in; left_channel_out=(int16_t) OutputValues_A[local_Sample_Counter]; 

} 

835 

20.6 Example 3—Real-time filter design 

right_channel_out = right_channel_in; // Only left channel is processed // Writes sample to the DAC play_sample(&left_channel_out, &right_channel_out); local_Sample_Counter++; if (local_Sample_Counter>=BLOCKSIZE) { // Wraps around and toggles the ping-pong buffer local_Sample_Counter = 0; PingPongState = (PingPongState+1) & 0x1; // toggles ping-pong state if (processing_trigger==2) { // Biquad is still running – an overrun error has occurred __BKPT(1); // Error } else { processing_trigger = 1; // Starts new biquad SCB->ICSR j= SCB_ICSR_PENDSVSET_Msk; // Triggers PendSV } } 

Sample_Counter = local_Sample_Counter; // saves new Sample_Counter return; } 

The FPGA platform I am using contains a Cortex-M33 processor running at 20MHz, and the measured processing loading is around 22%. Please note, the measured utilization ratio for the processor is more than the filter processing time because it also includes the clock cycles that are needed for handling the audio interface interrupt services. 

### 20.6.4 Use of an RTOS for handling filter processing 

In many applications, an RTOS is used to manage large numbers of processing tasks. Using the task prioritization capability of a modern RTOS, we can prioritize the filter processing over other less time-critical processing tasks and, by so doing, achieve the same effect as running the filter processing function using a low priority exception handler. 

In the following example, an RTX RTOS is used to demonstrate how the filter processing task is executed as one of the application threads in an RTOS environment. To allow the audio interface interrupt to trigger the execution of the filter processing task, an OS event is used. The filter processing task (i.e., “biquad_processing()” in the following code) waits for the OS event and proceeds when the OS event is received from the audio interface interrupt handler (i.e., the “I2S_Handler()”). 

836 

20. Using the Arm CMSIS-DSP library 

Real-time filter code using RTX, with filter processing running as a real-time application thread // ASN Filter Designer Professional v4.3.3 // Fri, 07 Feb 2020 18:41:12 GMT // ** Primary Filter (H1)** ////Band# Frequencies (kHz) Att/Ripple (dB) // 1 0.000, 2.400 0.001 // 2 12.000, 24.000 80.000 //// // Arithmetic = ’Floating Point (Single Precision)’; // Architecture = ’IIR’; // Structure = ’Direct Form II Transposed’; // Response = ’Lowpass’; // Method = ’Butterworth’; // Biquad = ’Yes’; // Stable = ’Yes’; // Fs = 48.0000; //kHz // Filter Order = 8; // ** ASN Filter Designer Automatic Code Generator ** // ** Deployment to Arm CMSIS-DSP Framework ** // Included Headers //#include "IOTKit_CM33_FP.h" #include "SMM_MPS2.h" #include "arm_math.h" // CMSIS-DSP library header #include "stdio.h" #include "cmsis_os2.h" extern uint8_t audio_init(void); // Initializes audio hardware extern void read_sample(int16_t *left, int16_t *right); extern void play_sample(int16_t *left, int16_t *right); void biquad_processing (void *arg); void report_utilization (void *arg); void I2S_Handler(void); osEventFlagsId_t evt_id; // message event id osThreadId_t biquad_tread_id; osThreadId_t report_tread_id; #define FLAGS_MSK1 0x00000001ul 

837 

20.6 Example 3—Real-time filter design 

const osThreadAttr_t report_thread1_attr = { .stack_size = 1024 // Creates the thread stack with a size of 1024 bytes }; #define BLOCKSIZE 32 #define NUM_SECTIONS_IIR 4 // ** IIR Direct Form II Transposed Biquad Implementation ** // y[n] = b0 * x[n] + w1 // w1 = b1 * x[n] + a1 * y[n] + w2 // w2 = b2 * x[n] + a2 * y[n] // IIR Coefficients // IIR Coefficients float32_t const static iirCoeffsf32[NUM_SECTIONS_IIR*5] = {// b0, b1, b2, a1, a2 0.0582619f, 0.1165237f, 0.0582619f, 1.0463270f, -0.2788444f, 0.0615639f, 0.1231277f, 0.0615639f, 1.1071010f, -0.3531238f, 0.0688354f, 0.1376707f, 0.0688354f, 1.2402050f, -0.5158056f, 0.0815540f, 0.1631081f, 0.0815540f, 1.4713260f, -0.7982877f }; float32_t static iirStatesf32[NUM_SECTIONS_IIR*5]; static arm_biquad_cascade_df2T_instance_f32 S; // Ping-pong buffer float32_t static InputValues_A[BLOCKSIZE]; float32_t static OutputValues_A[BLOCKSIZE]; float32_t static InputValues_B[BLOCKSIZE]; float32_t static OutputValues_B[BLOCKSIZE]; volatile int static PingPongState=0; // 0 = processing A, I/O = B // 1 = processing B, I/O = A volatile int static Sample_Counter=0; // Counts sample from 0 to BLOCKSIZE-1, // then triggers processing and returns to 0 volatile int static processing_trigger=0; // When it is set to 1, it starts processing // Sets to 2 during processing & returns to 0 when processing is completed // Samples from both audio channels int16_t static left_channel_in; int16_t static right_channel_in; int16_t static left_channel_out; int16_t static right_channel_out; 

Continued 

838 

20. Using the Arm CMSIS-DSP library 

// Number of cycle per block of samples #define CPUCYCLE_MAX ((20000000/48000)*BLOCKSIZE) static volatile uint32_t cycle_cntr=0; int main(void) { int i; osStatus_t status; // Initializes data in ping-pong buffer for (i=0;i<BLOCKSIZE;i++) { OutputValues_A[i] = 0.0f; OutputValues_B[i] = 0.0f; } // Initializes Biquads arm_biquad_cascade_df2T_init_f32 (&S, NUM_SECTIONS_IIR, &(iirCoeffsf32[0]), & (iirStatesf32[0])); osKernelInitialize(); // Initializes CMSIS-RTOS // creates thread for the biquad_processing biquad_tread_id=osThreadNew(biquad_processing, NULL, NULL); // Sets thread priority status = osThreadSetPriority (biquad_tread_id, osPriorityRealtime); if (status == osOK) { // Thread priority successfully changed } else { // Fails to set the priority __BKPT(0); } // creates a thread for reporting the processor’s utilization report_tread_id=osThreadNew(report_utilization, NULL, &report_thread1_attr); // Sets thread priority status = osThreadSetPriority (report_tread_id, osPriorityNormal); if (status == osOK) { // Thread priority successfully changed } else { // Fails to set the priority __BKPT(0); } 

evt_id = osEventFlagsNew(NULL); // Creates an event object (Cannot be called until osKernelInitialize is complete) 

839 

20.6 Example 3—Real-time filter design 

audio_init(); // Initializes the audio interface osKernelStart(); // Starts the thread execution while(1); 

} 

/************************************************************************/ /* Biquad thread. Triggers at 48KHz/BLOCKSIZE. */ /************************************************************************/ void biquad_processing (void *arg) { uint32_t flags; while (1) { flags = osEventFlagsWait (evt_id,FLAGS_MSK1,osFlagsWaitAny, osWaitForever); processing_trigger=2; // indicates the biquad is running MPS2_SECURE_FPGAIO->COUNTER=0; 

if (PingPongState==0) { 

arm_biquad_cascade_df2T_f32 (&S, &InputValues_A[0], &OutputValues_A[0], BLOCKSIZE); // performs filtering 

} else { 

arm_biquad_cascade_df2T_f32 (&S, &InputValues_B[0], &OutputValues_B[0], BLOCKSIZE); // performs filtering } // endif-if (PingPongState==0) processing_trigger=0; // returns to 0 cycle_cntr = MPS2_FPGAIO->COUNTER; } 

return; 

} 

/************************************************************************/ /* Processor load reporting thread */ /************************************************************************/ void report_utilization (void *arg) 

{ 

while (1) { osDelay(1000); printf ("cpu load=%d percent\n", (100*cycle_cntr/CPUCYCLE_MAX)); 

- } 

- } /************************************************************************/ /* I2S audio IRQ handler. Triggers at 48KHz. */ /************************************************************************/ void I2S_Handler(void) { int local_Sample_Counter; 

Continued 

840 

20. Using the Arm CMSIS-DSP library 

// Sample_Counter counts from 0 to BLOCKSIZE-1 local_Sample_Counter = Sample_Counter; // Reads sample from the ADC read_sample(&left_channel_in, &right_channel_in); if (PingPongState==0) { InputValues_B[local_Sample_Counter] = (float) left_channel_in; left_channel_out=(int16_t) OutputValues_B[local_Sample_Counter]; } else { InputValues_A[local_Sample_Counter] = (float) left_channel_in; left_channel_out=(int16_t) OutputValues_A[local_Sample_Counter]; } right_channel_out = right_channel_in; // Only left channel is processed // Writes sample to the DAC play_sample(&left_channel_out, &right_channel_out); local_Sample_Counter++; if (local_Sample_Counter>=BLOCKSIZE) { // Wraps around and toggles the ping-pong buffer local_Sample_Counter = 0; PingPongState = (PingPongState+1) & 0x1; // toggle ping-pong state if (processing_trigger==2) { // Biquad is still running – an overrun error has occurred __BKPT(1); // Error } else { processing_trigger = 1; // Start new biquad osEventFlagsSet(evt_id, FLAGS_MSK1); } } Sample_Counter = local_Sample_Counter; // saves new Sample_Counter return; } 

When running the filter processing task in an RTOS, you might see that the measured processing time needed to complete the filter processing task increases compared to the example that I mentioned in Section 20.6.3. This is because the measured processing time included the time it took to execute: 

- The audio interrupt services, 

- The processing of the audio filter, 

- The OS tick exception handler and, potentially, 

- Other real-time threads. 

841 

20.6 Example 3—Real-time filter design 

### 20.6.5 Stereo audio biquad filter 

In many audio applications, the filter processing tasks operate on stereo audio data. A simple way to handle this is to run the filter algorithm twice: once for the left channel and once for the right. To make the processing more efficient, the CMSIS-DSP library also provides a biquad filter function for processing stereo data. 

When dealing with stereo data, the values for the left and right channels interleave. As a result of combining the left and right channels, the input and output buffers are double the size. Because the same filtering is applied to both channels, the filter coefficients are shared between the two channels. The example stereo filter code is shown below: 

Stereo filter example—bare metal environment (i.e., no OS) 

// ASN Filter Designer Professional v4.3.3 // Fri, 07 Feb 2020 18:41:12 GMT // ** Primary Filter (H1)** ////Band# Frequencies (kHz) Att/Ripple (dB) // 1 0.000, 2.400 0.001 // 2 12.000, 24.000 80.000 //// // Arithmetic = ’Floating Point (Single Precision)’; // Architecture = ’IIR’; // Structure = ’Direct Form II Transposed’; // Response = ’Lowpass’; // Method = ’Butterworth’; // Biquad = ’Yes’; // Stable = ’Yes’; // Fs = 48.0000; //kHz // Filter Order = 8; // ** ASN Filter Designer Automatic Code Generator ** // ** Deployment to Arm CMSIS-DSP Framework ** // Included Headers #include "IOTKit_CM33_FP.h" #include "arm_math.h" // CMSIS-DSP library header extern uint8_t audio_init(void); // Initializes audio hardware extern void read_sample(int16_t *left, int16_t *right); extern void play_sample(int16_t *left, int16_t *right); void I2S_Handler(void); 

Continued 

#### 842 

20. Using the Arm CMSIS-DSP library 

#define BLOCKSIZE 32 #define NUM_SECTIONS_IIR 4 // ** IIR Direct Form II Transposed Biquad Implementation ** // y[n] = b0 * x[n] + w1 // w1 = b1 * x[n] + a1 * y[n] + w2 // w2 = b2 * x[n] + a2 * y[n] // IIR Coefficients // IIR Coefficients float32_t const static iirCoeffsf32[NUM_SECTIONS_IIR*5] = {// b0, b1, b2, a1, a2 0.0582619f, 0.1165237f, 0.0582619f, 1.0463270f, -0.2788444f, 0.0615639f, 0.1231277f, 0.0615639f, 1.1071010f, -0.3531238f, 0.0688354f, 0.1376707f, 0.0688354f, 1.2402050f, -0.5158056f, 0.0815540f, 0.1631081f, 0.0815540f, 1.4713260f, -0.7982877f }; 

float32_t static iirStatesf32[NUM_SECTIONS_IIR*5]; 

static arm_biquad_cascade_stereo_df2T_instance_f32 S; 

// Ping-pong buffer float32_t static InputValues_A[BLOCKSIZE*2]; float32_t static OutputValues_A[BLOCKSIZE*2]; float32_t static InputValues_B[BLOCKSIZE*2]; float32_t static OutputValues_B[BLOCKSIZE*2]; volatile int static PingPongState=0; // 0 = processing A, I/O = B // 1 = processing B, I/O = A 

volatile int static Sample_Counter=0; // Counts sample from 0 to BLOCKSIZE-1, // then triggers processing and returns to 0 volatile int static processing_trigger=0; // When it is set to 1, it starts processing // Sets to 2 during processing & returns to 0 when processing is completed // Samples from both audio channels int16_t static left_channel_in; int16_t static right_channel_in; int16_t static left_channel_out; int16_t static right_channel_out; int main(void) { int i; // Initializes data in ping-pong buffer 

843 

20.6 Example 3—Real-time filter design 

for (i=0;i<BLOCKSIZE*2;i++) { 

OutputValues_A[i] = 0.0f; OutputValues_B[i] = 0.0f; 

} 

// Initializes Biquads 

arm_biquad_cascade_stereo_df2T_init_f32 (&S, NUM_SECTIONS_IIR, &(iirCoeffsf32[0]), &(iirStatesf32[0])); 

audio_init(); // Initializes the audio interface 

- // Waits for I2S IRQs to occur and then services them. 

- // WFE sleep is used to allow power saving 

while(1){ 

if (processing_trigger>0) { 

// processing_trigger is set to 1 by I2S_Handler when a BLOCK of data is sampled processing_trigger=2; // indicates biquad is running if (PingPongState==0) { 

arm_biquad_cascade_stereo_df2T_f32 (&S, &InputValues_A[0], &OutputValues_A[0], BLOCKSIZE); // performs filtering 

- } else { 

arm_biquad_cascade_stereo_df2T_f32 (&S, &InputValues_B[0], &OutputValues_B[0], BLOCKSIZE); // performs filtering } // endif-if (PingPongState==0) processing_trigger=0; // returns state to 0 

- } // end-if (processing_trigger!=0) 

__WFE(); // enters sleep when the processing is finished 

###### } 

} 

- /************************************************************************/ /* I2S audio IRQ handler. Triggers at 48KHz. */ /************************************************************************/ void I2S_Handler(void) { 

int local_Sample_Counter; 

// Sample_Counter counts from 0 to BLOCKSIZE-1 local_Sample_Counter = Sample_Counter*2; 

- // Reads sample from the ADC read_sample(&left_channel_in, &right_channel_in); 

if (PingPongState==0) { 

Continued 

844 

20. Using the Arm CMSIS-DSP library 

InputValues_B[local_Sample_Counter ] = (float) left_channel_in; InputValues_B[local_Sample_Counter+1] = (float) right_channel_in; left_channel_out =(int16_t) OutputValues_B[local_Sample_Counter]; right_channel_out=(int16_t) OutputValues_B[local_Sample_Counter+1]; } else { InputValues_A[local_Sample_Counter] = (float) left_channel_in; InputValues_A[local_Sample_Counter+1] = (float) right_channel_in; left_channel_out =(int16_t) OutputValues_A[local_Sample_Counter]; right_channel_out=(int16_t) OutputValues_A[local_Sample_Counter+1]; } // Writes sample to the DAC play_sample(&left_channel_out, &right_channel_out); 

local_Sample_Counter=local_Sample_Counter+2; if (local_Sample_Counter>=(2*BLOCKSIZE)) { 

- // Wraps around and toggles the ping-pong buffer local_Sample_Counter = 0; PingPongState = (PingPongState+1) & 0x1; // toggles the ping-pong state if (processing_trigger==2) { 

- // Biquad is still running – an overrun error has occurred 

__BKPT(1); // Error 

} else { processing_trigger = 1; // Starts new biquad } } Sample_Counter = local_Sample_Counter>>1; // saves new Sample_Counter return; } 

### 20.6.6 Alternative buffer arrangement 

The real-time filter examples detailed so far have used two pairs of input and output buffers, i.e., a total of four blocks of buffer space. In microcontroller systems with a small amount of SRAM space, it is often desirable to reduce the SRAM usage. To achieve this, an alternative buffer arrangement can be used and an example of this is shown in Fig. 20.21. In this alternative buffer arrangement: 

- Three buffers are used. 

- Each buffer is used for both input and output. 

- The buffer system operates with three states (different from the previous examples that use two Ping-pong states). 

845 

20.7 How to determine the implemented instruction set features in a Cortex-M33 based system 



<!-- Start of picture text -->
Time Buffer A Buffer B Buffer C<br>een .<br>eS TTT<br>New data ‘Output<br>sample<br>\ \ Newsamplenitytut = j}  r<br>pe<br>Tee=<br>New data ‘Output<br>sample<br><!-- End of picture text -->

FIG. 20.21 An alternative buffer arrangement for real-time filters. 

Although this design can reduce memory usage, the software could be harder to debug because each buffer is used for both inputs and outputs. 

### 20.7 How to determine the implemented instruction set features in a Cortex-M33 based system 

Since the FPU and DSP extensions are optional in the Cortex-M33 processor, a Cortex-M33based device might not have these extensions implemented. Normally, information about the implemented features should be available from the device’s specification/datasheet. In some instances, though, you might need to determine what is implemented in the chip via a debug connection or by executing software on the system. 

The Cortex-M33 processor includes a number of read-only registers to allow software or a debugger to determine what instruction set features have been implemented (Table 20.2). 

In addition to directly reading the registers listed in Table 20.2, the CMSIS-CORE has a function call for determining what type of FPU has been implemented. This function is: 

uint32_t SCB_GetFPUTyper(void); 

and returns the following values: 

- . 0—no FPU is implemented 

- : 1—Single precision FPU is implemented 

- : 2—Double + single precision FPU is implemented 

846 

20. Using the Arm CMSIS-DSP library 

TABLE 20.2 Registers that can be used for determining the implemented instruction set in the Cortex-M33 processor. 

|Register|Address<br>(Non-secure<br>alias)|CMSIS-CORE<br>symbol|Description|
|---|---|---|---|
|ID_ISAR1|0xE000ED64<br>(0xE002ED64)|SCB-><br>ID_ISAR[1]<br>SCB_NS-><br>ID_ISAR[1]|Instruction Set Attribute Register 1<br>• If the DSP Extension is not implemented, this register reads as<br>0x02211000.<br>• When the DSP Extension is implemented, this register reads as<br>0x02212000.|
|ID_ISAR2|0xE000ED68<br>(0xE002ED68)|SCB-><br>ID_ISAR[2]<br>SCB_NS-><br>ID_ISAR[2]|Instruction Set Attribute Register 2<br>• With bits [11:8] masked, if the DSP Extension is not<br>implemented, this register reads as 0x20112032.<br>• With bits [11:8] masked, when the DSP Extension is<br>implemented, this register reads as 0x20232032.|
|ID_ISAR3|0xE000ED6C<br>(0xE002ED6C)|SCB-><br>ID_ISAR[3]<br>SCB_NS-><br>ID_ISAR[3]|Instruction Set Attribute Register 3<br>• If the DSP Extension is not implemented, this register reads as<br>0x01111110.<br>• When the DSP Extension is implemented, this register reads as<br>0x01111131.|
|MVFR0|0xE000EF40<br>(0xE002EF40)|FPU->MVFR0<br>FPU_NS-><br>MVFR0|Media and VFP Feature Register 0<br>• If the Floating-point Extension is not implemented this register<br>reads as 0x00000000.<br>• Where only a single-precision floating-point unit is supported<br>this register reads as 0x10110021.<br>• Where a single and double-precision floating-point unit are<br>supported this register reads as 0x10110221. (This<br>configuration is not supported in the Cortex-M33 processor)|
|MVFR1|0xE000EF44<br>(0xE002EF44)|FPU->MVFR1<br>FPU_NS-><br>MVFR1|Media and VFP Feature Register 1<br>• If the floating-point extension is not implemented this register<br>reads as 0x00000000.<br>• Where only a single-precision floating-point unit is supported<br>this register reads as 0x11000011.<br>• Where single and double-precision floating-point are<br>supported this register reads as 0x12000011. (This<br>configuration is not supported in the Cortex-M33 processor)|
|MVFR2|0xE000EF48<br>(0xE002EF48)|FPU->MVFR2<br>FPU_NS-><br>MVFR2|Media and VFP Feature Register 2<br>• If the floating-point extension is not implemented this register<br>reads as 0x00000000.<br>• When the floating-point unit is implemented this register reads<br>as 0x00000040.|



847 

References 

### References 

- [1] CMSIS-DSP library documentation. https://arm-software.github.io/CMSIS_5/DSP/html/index.html. 

- [2] The Advanced Solution Nederland web site has a good article on filter design. Classical IIR filter design: a practical guide. http://www.advsolned.com/iir-filters-a-practical-guide/. 

