
-- NUM_SCAN_ELEMENTS
#ifdef SCAN_DYNAMIC_HISTOGRAM_SIZE
uniform uint histogramSize;
#define NUM_SCAN_ELEMENTS histogramSize
#else
#define NUM_SCAN_ELEMENTS SCAN_HISTOGRAM_SIZE
#endif

--------------
--- Serial scan: Use a single thread to compute the prefix sum of the histogram.
--- This should only be used for small histograms, as it is not parallelized.
--------------
-- serial.cs
#include regen.stages.compute.defines
#include regen.compute.prefix-scan.NUM_SCAN_ELEMENTS
// - [read/write] the global histogram. NOTE: input should be counts, output will be (global) offsets.
buffer uint in_globalHistogram[];

void main() {
    uint globalID = gl_GlobalInvocationID.x;
    if (globalID == 0) {
        uint sum = 0;
        for (uint i = 0; i < NUM_SCAN_ELEMENTS; ++i) {
            uint h_i = in_globalHistogram[i];
            in_globalHistogram[i] = sum;
            sum += h_i;
        }
    }
}

--------------
--- Parallel scan: Use a parallel prefix sum algorithm to compute the prefix sum of the histogram.
--- Should be dispatchd with a single work group having a thread for each element in the histogram.
--- This will work for mid-sized histograms, as it is parallelized. However, the number of threads
--- per work group is limited.
--- The histogram size for radix sort is NUM_WORK_GROUPS * NUM_BUCKETS.
--- NUM_WORK_GROUPS can get large, e.g. with 1.000.000 keys and work group size of 256,
--- we have 3907 work groups, for 4bit radix sort we have 16 buckets,
--- so the histogram size is 3907 * 16 = 62512 times 32-bit which is ~250kB
--- and might be too big for shared memory which is often capped to 64kB.
--- So as an estimate with group size 256, only use shared memory sum up to ~100.000 keys,
--- and with group size 512, only use shared memory sum up to ~200.000 keys.
--------------
-- parallel.cs
#include regen.stages.compute.defines
#include regen.compute.prefix-scan.NUM_SCAN_ELEMENTS
// - [read/write] the global histogram. NOTE: input should be counts, output will be (global) offsets.
buffer uint in_globalHistogram[];
shared uint sh_offsets[NUM_SCAN_THREADS];

void main() {
    uint tid = gl_LocalInvocationID.x;

    // Load input into shared memory
    if (tid < NUM_SCAN_ELEMENTS) {
        sh_offsets[tid] = in_globalHistogram[tid];
    } else {
        sh_offsets[tid] = 0; // pad with 0s if over histogram size
    }
    barrier();

    // === Upsweep (reduce) ===
    for (uint offset = 1; offset < NUM_SCAN_THREADS; offset *= 2) {
        uint index = (tid + 1) * offset * 2 - 1;
        if (index < NUM_SCAN_THREADS) {
            sh_offsets[index] += sh_offsets[index - offset];
        }
        barrier();
    }

    // === Set last element to zero (for exclusive scan) ===
    if (tid == 0) {
        sh_offsets[NUM_SCAN_THREADS - 1] = 0;
    }
    barrier();

    // === Downsweep ===
    for (uint offset = NUM_SCAN_THREADS / 2; offset > 0; offset /= 2) {
        uint index = (tid + 1) * offset * 2 - 1;
        if (index < NUM_SCAN_THREADS) {
            uint t = sh_offsets[index - offset];
            sh_offsets[index - offset] = sh_offsets[index];
            sh_offsets[index] += t;
        }
        barrier();
    }

    // Write result back
    if (tid < NUM_SCAN_ELEMENTS) {
        in_globalHistogram[tid] = sh_offsets[tid];
    }
}

--------------
--- Each workgroup scans WG_SIZE elements and:
---     - Writes scanned output to in_globalHistogram.
---     - Writes total sum of block to a blockSums[] array.
--------------
-- local.cs
// num_work_units(HISTOGRAM_SIZE)
// layout(local_size_x = (WG_SIZE=512)) in;
// Note: the number of blocks is computed as: ceil(HISTOGRAM_SIZE / WG_SIZE)
#include regen.stages.compute.defines
#include regen.compute.prefix-scan.NUM_SCAN_ELEMENTS

// The global histogram, it reflects global offsets for each bucket and workgroup.
buffer uint in_globalHistogram[];
buffer uint in_blockOffsets[];
shared uint sh_temp[CS_LOCAL_SIZE_X];

void main() {
    uint tid = gl_LocalInvocationID.x;
    uint gid = gl_GlobalInvocationID.x;

    // Load to shared memory
    sh_temp[tid] = (gid < NUM_SCAN_ELEMENTS) ? in_globalHistogram[gid] : 0;
    barrier();

    // Upsweep (reduce)
    for (uint offset = 1; offset < CS_LOCAL_SIZE_X; offset <<= 1) {
        uint i = (tid + 1) * offset * 2 - 1;
        if (i < CS_LOCAL_SIZE_X)
            sh_temp[i] += sh_temp[i - offset];
        barrier();
    }

    if (tid == 0) {
        // Save total sum
        uint blockID = gl_WorkGroupID.x;
        in_blockOffsets[blockID] = sh_temp[CS_LOCAL_SIZE_X - 1];
        sh_temp[CS_LOCAL_SIZE_X - 1] = 0;
    }
    barrier();

    // Downsweep
    for (uint offset = CS_LOCAL_SIZE_X >> 1; offset > 0; offset >>= 1) {
        uint i = (tid + 1) * offset * 2 - 1;
        if (i < CS_LOCAL_SIZE_X) {
            uint t = sh_temp[i - offset];
            sh_temp[i - offset] = sh_temp[i];
            sh_temp[i] += t;
        }
        barrier();
    }

    // Store back result
    if (gid < NUM_SCAN_ELEMENTS)
        in_globalHistogram[gid] = sh_temp[tid];
}

-- global.cs
// NUM_BLOCKS = ceil(HISTOGRAM_SIZE / WG_SIZE)
// NUM_BLOCKS2 = nextPowerOfTwo(NUM_BLOCKS)
// num_work_units(NUM_BLOCKS2)
// layout(local_size_x = NUM_BLOCKS2) in;
#include regen.stages.compute.defines

buffer uint in_blockOffsets[];
shared uint sh_temp[CS_LOCAL_SIZE_X];

#ifdef SCAN_DYNAMIC_HISTOGRAM_SIZE
#define NUM_SCAN_BLOCKS numBlocks
#else
#define NUM_SCAN_BLOCKS SCAN_NUM_BLOCKS
#endif

void main() {
    uint tid = gl_LocalInvocationID.x;
    if (tid >= NUM_SCAN_BLOCKS) return;

    sh_temp[tid] = in_blockOffsets[tid];
    barrier();

    for (uint offset = 1; offset < CS_LOCAL_SIZE_X; offset <<= 1) {
        uint i = (tid + 1) * offset * 2 - 1;
        if (i < CS_LOCAL_SIZE_X)
            sh_temp[i] += sh_temp[i - offset];
        barrier();
    }

    if (tid == 0)
        sh_temp[CS_LOCAL_SIZE_X - 1] = 0;
    barrier();

    for (uint offset = CS_LOCAL_SIZE_X >> 1; offset > 0; offset >>= 1) {
        uint i = (tid + 1) * offset * 2 - 1;
        if (i < CS_LOCAL_SIZE_X) {
            uint t = sh_temp[i - offset];
            sh_temp[i - offset] = sh_temp[i];
            sh_temp[i] += t;
        }
        barrier();
    }

    in_blockOffsets[tid] = sh_temp[tid];
}

-- distribute.cs
// num_work_units(HISTOGRAM_SIZE)
// layout(local_size_x = (WG_SIZE=512)) in;
// Note: the number of blocks is computed as: ceil(HISTOGRAM_SIZE / WG_SIZE)
#include regen.stages.compute.defines
#include regen.compute.prefix-scan.NUM_SCAN_ELEMENTS
// The global histogram, it reflects global offsets for each bucket and workgroup.
buffer uint in_globalHistogram[];
buffer uint in_blockOffsets[];

void main() {
    uint gid = gl_GlobalInvocationID.x;
    uint blockID = gl_WorkGroupID.x;
    if (gid >= NUM_SCAN_ELEMENTS) return;
    in_globalHistogram[gid] += in_blockOffsets[blockID];
}
