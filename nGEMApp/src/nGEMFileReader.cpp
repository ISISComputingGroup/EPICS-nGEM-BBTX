/*************************************************************************\ 
* Copyright (c) 2023 Science and Technology Facilities Council (STFC), GB. 
* All rights reverved. 
* This file is distributed subject to a Software License Agreement found 
* in the file LICENSE.txt that is included with this distribution. 
\*************************************************************************/ 

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <cstdlib>
#include <cstring>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <assert.h>

#include <napi.h>

// swap endian
static uint64_t swap_uint64(uint64_t val)
{
    val = ((val << 8) & 0xFF00FF00FF00FF00ULL) | ((val >> 8) & 0x00FF00FF00FF00FFULL);
    val = ((val << 16) & 0xFFFF0000FFFF0000ULL) | ((val >> 16) & 0x0000FFFF0000FFFFULL);
    return (val << 32) | (val >> 32);
}

// each nGEM event is 128 bits, 2 64 bit words. It is big endian format
// so must be byte swapped on Intel PC; also structure bit fields are
// here written in reverse order to that in manual due to endian swap

struct Split64
{
    uint64_t w[2];
};

// continuation code ID for second 64bit word (CID)
#define CID_VALUE 0x4f

struct GenericEvent
{
    uint64_t T0ID : 24; // T0 identifier
    uint64_t unused2 : 32;
    uint64_t CID : 8; // continuation code
    uint64_t unused1 : 56;
    uint64_t ID : 8;
    bool check() const { return CID == CID_VALUE; }
    std::ostream& print(std::ostream& os) const { os << "GenericEvent"; return os; }
};

// indicate time 0, new t0id
struct T0FrameEvent
{
    uint64_t T0ID : 24; // T0 identifier
    uint64_t EC : 32; // Event count
    uint64_t CID : 8; // continuation code
    uint64_t TL : 24; // Total loss
    uint64_t EL : 20; // Event loss count
    uint64_t FL : 12; // Frame Loss count
    uint64_t ID : 8; // 0x4E 
    bool check() const { return ID == id && CID == CID_VALUE; }
    static const int id = 0x4E;
    std::ostream& print(std::ostream& os) const { os << "T0FrameEvent: T0ID=" << T0ID << " EC=" << EC; return os; }
};

struct DeviceTimeEvent
{
    uint64_t T0ID : 24; // T0 identifier
    uint64_t R : 32; // not used, all zero
    uint64_t CID : 8; // continuation code
    uint64_t US : 11; // time by 25ns unit
    uint64_t SS : 15; // time fraction, divide by 32768 to get sec
    uint64_t S : 12; // seconds from origin, 1-Jan-2008
    uint64_t ID : 8; // 0x4C 
    bool check() const { return ID == id && CID == CID_VALUE && R == 0; }
    static const int id = 0x4C;
    std::ostream& print(std::ostream& os) const { os << "DeviceTimeEvent: T0ID=" << T0ID << " TIME=" << getTime(); return os; }
    std::string getTime() const
    {
        static int first_call = 1;
        static time_t ref_time;
        if (first_call)
        {
            struct tm tms;
            memset(&tms, 0, sizeof(tms));
            tms.tm_mday = 1;
            tms.tm_year = 108;
            tms.tm_isdst = -1;
            ref_time = mktime(&tms);  // offset of 1-JAN-2008 from 1-JAN-1970
            first_call = 0;
        }
        char buffer[64], buffer2[64];
        time_t the_time = ref_time + S;
        struct tm* ptm = localtime(&ref_time);
        strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", ptm);
        double frac_sec = static_cast<double>(US) / 40000000.0 + static_cast<double>(SS) / 32768.0;
        sprintf(buffer2, "%s %.9f", buffer, frac_sec); // time and nanoseconds
        return buffer2;
    }
};

struct PulseNumberEvent
{
    uint64_t T0ID : 24; // T0 identifier
    uint64_t R2 : 32; // not used, all zero
    uint64_t CID : 8; // continuation code
    uint64_t K : 32; // pulse number
    uint64_t R1 : 8; // not used, all zero
    uint64_t M : 8; // Module number
    uint64_t C : 8; // Crate number
    uint64_t ID : 8; // 0x4B 
    bool check() const { return ID == id && CID == CID_VALUE && R1 == 0 && R2 == 0; }
    static const int id = 0x4B;
    std::ostream& print(std::ostream& os) const { os << "PulseNumberEvent: T0ID=" << T0ID << " K=" << K; return os; }
};

struct CoincidenceEvent
{
    uint64_t T0ID : 24; // T0 identifier
    uint64_t SY : 10; // Integrated time of the cluster on Y side 5ns pixel unit
    uint64_t DY : 6; // Time lag from first detection to last detection on Y side 5ns unit
    uint64_t SX : 10; // Integrated time of the cluster on X side 5ns pixel unit
    uint64_t DX : 6; // Time lag from first detection to last detection on X side 5ns unit
    uint64_t CID : 8; // continuation code
    uint64_t LY : 7; // Y position of pixel detected last
    uint64_t FY : 7; // Y position of pixel detected first
    uint64_t LX : 7; // X position of pixel detected last
    uint64_t FX : 7; // X position of pixel detected first
    uint64_t TOF : 28; // TOF in ns
    uint64_t ID : 8; // 0x47
    bool check() const { return ID == id && CID == CID_VALUE; }
    static const int id = 0x47;
    std::ostream& print(std::ostream& os) const { os << "CoincidenceEvent: T0ID=" << T0ID << " TOF(us)=" << TOF / 1000.0 << " POS=" << FX << "," << FY << "," << LX << "," << LY; return os; }
    uint64_t X() const { return (FX + LX) / 2; }
    uint64_t Y() const { return (FY + LY) / 2; }
    int getIndex() const { return X() + (Y() << 7); } // effective index for 128x128 grid, X + 128 * Y
};

union EventUnion
{
    GenericEvent ge;
    T0FrameEvent t0;
    DeviceTimeEvent dt;
    PulseNumberEvent pn;
    CoincidenceEvent ce;
    Split64 s;
};

// return array index else -1 if not there
static int findBin(const std::vector<float>& tcb, float val)
{
    int elem = std::distance(tcb.begin(), std::upper_bound(tcb.begin(), tcb.end(), val));
    if (elem == 0 || elem == tcb.size())
    {
        return -1;
    }
    else
    {
        return elem - 1;
    }
}

int main(int argc, char* argv[])
{
    EventUnion eu, euBE;
    uint64_t nread = 0, nevents = 0, nt0 = 0, nt0_check = 0;
    int n, histx[128], histy[128], hist2d[128][128];
    int last_t0 = -1;
    assert(sizeof(EventUnion) == 16);
    memset(histx, 0, sizeof(histx));
    memset(histy, 0, sizeof(histy));
    memset(hist2d, 0, sizeof(hist2d));
#ifdef _WIN32
    const char* open_mode = "rbS"; // optimise file cache for sequential access
#else
    const char* open_mode = "rb";
#endif /* _WIN32 */
    if (argc < 4)
    {
        std::cerr << "No Filenames given: input output template" << std::endl;
        return 1;
    }
    const char *input_file = argv[1], *output_file = argv[2], *template_name = argv[3];
    std::string template_file = std::string("\\\\isis\\inst$\\Kits$\\CompGroup\\freddie\\nGEM\\ngem_template_") + template_name + ".nxs";

#ifdef _WIN32
    if (CopyFile(template_file.c_str(), output_file, 0) == 0)
#else
    if (true) // TODO: linux equivalent of CopyFile
#endif
    {
        std::cerr << "copy error for " << template_file << " to " << output_file << std::endl;
        return 1;
    }
    NXhandle nxhandle = NULL;
    if (NXopen(output_file, NXACC_RDWR, &nxhandle) != NX_OK)
    {
        std::cerr << "nxopen error for output file " << output_file << std::endl;
        return 1;
    }
    int rank, datatype, dimensions[3];
    NXopenpath(nxhandle, "/raw_data_1/detector_1/time_of_flight");
    NXgetinfo(nxhandle, &rank, dimensions, &datatype);
    std::vector<float> tcb(dimensions[0]);
    NXgetdata(nxhandle, &(tcb[0]));
    int ntc = dimensions[0] - 1; // tcb is boundaries, hence is size ntc + 1 
    int32_t* counts = new int32_t[128 * 128 * ntc];
    memset(counts, 0, 128 * 128 * ntc * sizeof(int32_t));
    FILE* f = fopen(input_file, open_mode);
    if (f == NULL)
    {
        std::cerr << "Input file \"" << input_file << "\" does not exist" << std::endl;
        return 1;
    }
    struct stat st;
    if (fstat(fileno(f), &st) != 0 || st.st_size % 16 != 0)
    {
        std::cerr << "File size error" << std::endl;
        return 1;
    }
    int max_events = st.st_size / 16;
    int* event_id = new int[max_events];
    float* event_time_offset = new float[max_events];
    float min_tof = 1000000000, max_tof = 0;
    std::vector<int> event_index;
    std::cerr << "Number of events (T0 + coincidence) = " << max_events << std::endl;
    while (true)
    {
        if (feof(f))
        {
            std::cerr << "Read finished" << std::endl;
            break;
        }
        n = fread(&euBE, sizeof(euBE), 1, f);
        if (n == 0)
        {
            std::cerr << "Read finished" << std::endl;
            break;
        }
        else if (n != 1)
        {
            std::cerr << "Read error" << std::endl;
            break;
        }
        // correct for big endian on-disk format
        eu.s.w[0] = swap_uint64(euBE.s.w[1]);
        eu.s.w[1] = swap_uint64(euBE.s.w[0]);
        ++nread;
        if (eu.ge.T0ID != last_t0)
        {
            event_index.push_back(nevents);
            last_t0 = eu.ge.T0ID;
        }
        if (eu.t0.check())
        {
            //			eu.t0.print(std::cerr);
            //		    std::cerr << std::endl;
            ++nt0;
        }
        else if (eu.dt.check())
        {
            eu.dt.print(std::cerr);
        }
        else if (eu.pn.check())
        {
            eu.pn.print(std::cerr);
        }
        else if (eu.ce.check())
        {
            ++(histx[eu.ce.X()]);
            ++(histy[eu.ce.Y()]);
            ++(hist2d[eu.ce.X()][eu.ce.Y()]);
            int ind = eu.ce.getIndex();
            float tof_us = eu.ce.TOF / 1000.0;
            if (tof_us < min_tof)
                min_tof = tof_us;
            if (tof_us > max_tof)
                max_tof = tof_us;
            event_id[nevents] = ind + 1; // spectrum number
            event_time_offset[nevents] = tof_us;
            ++nevents;
            int bin = findBin(tcb, tof_us);
            if (bin != -1)
            {
                ++(counts[ind * ntc + bin]);
            }
            //			eu.ce.print(std::cerr);
            //		    std::cerr << std::endl;
        }
        else
        {
            std::cerr << "Data error" << std::endl;
            break;
        }
    }
    fclose(f);
    std::cerr << "Coincidence events = " << nevents << ", T0 events = " << nt0 << std::endl;
    std::cerr << "TOF range seen in file (us): " << min_tof << " to " << max_tof << std::endl;
    std::cerr << "Rebinned to " << ntc << " bins between " << tcb[0] << " and " << tcb[ntc] << std::endl;
#if 0
    std::fstream fs;
    fs.open("histx.csv", std::ios::out);
    for (int i = 0; i < 128; ++i)
    {
        fs << i << "," << histx[i] << std::endl;
    }
    fs.close();
    fs.open("histy.csv", std::ios::out);
    for (int i = 0; i < 128; ++i)
    {
        fs << i << "," << histy[i] << std::endl;
    }
    fs.close();
    fs.open("histxy.csv", std::ios::out);
    for (int i = 0; i < 128; ++i)
    {
        for (int j = 0; j < 128; ++j)
        {
            fs << hist2d[i][j] << (j < 127 ? "," : "");
        }
        fs << std::endl;
    }
    fs.close();
#endif
    float fzero(0.0);
    int izero(0);
    int frames = nt0;
    NXopenpath(nxhandle, "/raw_data_1/run_number");
    NXputdata(nxhandle, &izero);
    NXopenpath(nxhandle, "/raw_data_1/duration");
    NXputdata(nxhandle, &izero);
    NXopenpath(nxhandle, "/raw_data_1/good_frames");
    NXputdata(nxhandle, &frames);
    NXopenpath(nxhandle, "/raw_data_1/raw_frames");
    NXputdata(nxhandle, &frames);
    NXopenpath(nxhandle, "/raw_data_1/proton_charge");
    NXputdata(nxhandle, &fzero);
    NXopenpath(nxhandle, "/raw_data_1/proton_charge_raw");
    NXputdata(nxhandle, &fzero);
    NXopenpath(nxhandle, "/raw_data_1/detector_1/counts");
    NXputdata(nxhandle, counts);
    NXclose(&nxhandle);
    return 0;
}
