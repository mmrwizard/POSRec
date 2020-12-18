#ifndef TICTOC_HPP
#define TICTOC_HPP

#ifdef WITH_TICTOC

#include <stdio.h>

#include <map>
#include <string>

#ifdef _MSC_VER

#include < time.h >
#include < windows.h >

#if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
#define DELTA_EPOCH_IN_MICROSECS 11644473600000000Ui64
#else
#define DELTA_EPOCH_IN_MICROSECS 11644473600000000ULL
#endif

struct timezone {
    int tz_minuteswest; /* minutes W of Greenwich */
    int tz_dsttime;     /* type of dst correction */
};

int gettimeofday(struct timeval *tv, struct timezone *tz) {
    FILETIME ft;
    unsigned __int64 tmpres = 0;
    static int tzflag = 0;

    if (NULL != tv) {
        GetSystemTimeAsFileTime(&ft);

        tmpres |= ft.dwHighDateTime;
        tmpres <<= 32;
        tmpres |= ft.dwLowDateTime;

        tmpres /= 10; /*convert into microseconds*/
        /*converting file time to unix epoch*/
        tmpres -= DELTA_EPOCH_IN_MICROSECS;
        tv->tv_sec = (long)(tmpres / 1000000UL);
        tv->tv_usec = (long)(tmpres % 1000000UL);
    }

    if (NULL != tz) {
        if (!tzflag) {
            _tzset();
            tzflag++;
        }
        tz->tz_minuteswest = _timezone / 60;
        tz->tz_dsttime = _daylight;
    }

    return 0;
}
#endif

namespace {

namespace med {

// Gets current time down to microseconds
struct timeval get_tick() {
    struct timeval time;
    gettimeofday(&time, NULL);
    return time;
}

typedef long line_id;

typedef struct timestamp {
    line_id line;
    struct timeval time;
} timestamp;

/**
 * Timer
 */
class Timer {
public:
    Timer() : filename_(""), display_(false) {}
    Timer(std::string filename, bool display = true) : filename_(filename), display_(display) {
        this->tic_time_.line = -1;
    }
    Timer(const Timer &t) : filename_(t.filename_), display_(t.display_), tic_time_(t.tic_time_) {}
    void tic(line_id line, struct timeval time) {
        this->tic_time_.line = line;
        this->tic_time_.time = time;
    };
    void toc(line_id line, struct timeval time) {
        if (this->display_ && tic_time_.line >= 0) {
            printf("%s [%5d,%5d]   elapsed: %10.3f s  %10.3f ms  %10d us\n", this->filename_.c_str(), tic_time_.line,
                   line, ((time.tv_sec - tic_time_.time.tv_sec) + (time.tv_usec - tic_time_.time.tv_usec) / 1000000.0),
                   ((time.tv_sec - tic_time_.time.tv_sec) * 1000. + (time.tv_usec - tic_time_.time.tv_usec) / 1000.0),
                   ((time.tv_sec - tic_time_.time.tv_sec) * 1000000 + time.tv_usec - tic_time_.time.tv_usec));
        }
    };
    void tictoc(line_id line, struct timeval time) {
        this->toc(line, time);
        this->tic(line, time);
    };
    void set_display(bool display) { this->display_ = display; }

public:
    std::string filename_;
    bool display_;
    int max_hist_length_;
    timestamp tic_time_;
};

/**
 * Timer manager
 */
class TimerManager {

public:
    static std::map<std::string, Timer> timer_map;

    static void init(std::string filename, bool display = true) {
        if (timer_map.find(filename) != timer_map.end()) {
            return;
        }
        timer_map[filename] = Timer(filename, display);
    }

    static void set_display(bool display) {
        for (std::map<std::string, Timer>::iterator it = timer_map.begin(); it != timer_map.end(); ++it) {
            it->second.set_display(display);
        }
    }

    static void tic(std::string filename, line_id line) {
        init(filename);
        timer_map[filename].tic(line, get_tick());
    }
    static void toc(std::string filename, line_id line) {
        init(filename);
        timer_map[filename].toc(line, get_tick());
    }
    static void tictoc(std::string filename, line_id line) {
        init(filename);
        timer_map[filename].tictoc(line, get_tick());
    }
};

std::map<std::string, Timer> TimerManager::timer_map;

} // namespace med
} // namespace

#endif

#ifdef WITH_TICTOC
#define LOCATION std::string(__FILE__) + " @ " + std::string(__FUNCTION__)
#define TICTOC_DISPLAY med::TimerManager::set_display(true);
#define TICTOC_NODISPLAY med::TimerManager::set_display(false);
#define TIC                                                                                                            \
    med::TimerManager::init(LOCATION);                                                                                 \
    med::TimerManager::tic(LOCATION, __LINE__);
#define TOC                                                                                                            \
    med::TimerManager::init(LOCATION);                                                                                 \
    med::TimerManager::toc(LOCATION, __LINE__);
#define TICTOC                                                                                                         \
    med::TimerManager::init(LOCATION);                                                                                 \
    med::TimerManager::tictoc(LOCATION, __LINE__);
#else
#define TICTOC_DISPLAY
#define TICTOC_NODISPLAY
#define TIC
#define TOC
#define TICTOC
#endif

#endif
