
#ifndef EV3_SECTION_TRACER_H_
#define EV3_SECTION_TRACER_H_

#include <math.h>
#include <Clock.h>
#include "GuageManager.h"
#include "LineMonitor.h"
#include "GyroSensor.h"
#include "PID.h"
#include "Motor.h"
#include "Odometer.h"
#include "Section.h"
#include "LineTracer.h"
#include "TailController.h"
#include "TailWalker.h"
#include "stdlib.h"

class SectionTracer
{
public:
    SectionTracer(GuageManager *robot_info,
                  Section *section,
                  LineTracer *line_tracer);

    bool run();

private:
    GuageManager *m_robot_info;
    Section *m_section;
    LineTracer *m_line_tracer;

    bool m_is_initialized;
    int m_section_num = 0;
    // int m_section_num = 14;
};

#endif // EV3_SECTION_TRACER_H_
