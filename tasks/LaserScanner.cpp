/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "LaserScanner.hpp"

#include <rtt/extras/FileDescriptorActivity.hpp>
#include <aggregator/TimestampEstimator.hpp>

using namespace velodyne_lidar;

    LaserScanner::LaserScanner(std::string const& name)
: LaserScannerBase(name)
{
    ir_frame = 0;
    ir_interp_frame = 0;
    range_frame = 0;
    range_interp_frame = 0;
    azimuth_frame = 0;
}

    LaserScanner::LaserScanner(std::string const& name, RTT::ExecutionEngine* engine)
: LaserScannerBase(name, engine)
{
}

LaserScanner::~LaserScanner()
{
}

bool LaserScanner::isScanComplete(const base::Angle& current_angle, const LaserScanner::LaserHeadVariables& laser_vars) const
{    
    if(laser_vars.horizontal_scan_count > 1)
    {
        if(current_angle == laser_vars.output_scan.horizontal_scans[0].horizontal_angle ||
                current_angle.isInRange(laser_vars.output_scan.horizontal_scans[0].horizontal_angle, 
                    laser_vars.output_scan.horizontal_scans[laser_vars.horizontal_scan_count-1].horizontal_angle))
        {
            return true;
        }
    }
    return false;
}

void LaserScanner::handleHorizontalScan(const velodyne_fire_t& horizontal_scan, LaserScanner::LaserHeadVariables& laser_vars)
{
    base::Angle scan_angle = base::Angle::fromDeg(((double)horizontal_scan.rotational_pos) * 0.01);
    if(isScanComplete(scan_angle, laser_vars))
    {
        // resize the array of all scans to the actual count
        laser_vars.output_scan.horizontal_scans.resize(laser_vars.horizontal_scan_count);

        // interpolate time between the actual and the last sample for all vertical scans
        base::Time last_output_time = laser_vars.output_scan.time;
        laser_vars.output_scan.time = laser_vars.timestamp_estimator->update(laser_vars.last_sample_time);

        if(last_output_time.microseconds == 0 || last_output_time > laser_vars.output_scan.time)
            last_output_time = laser_vars.output_scan.time;

        double time_between_scans_in_microseconds = ((double)(laser_vars.output_scan.time - last_output_time).microseconds) / (double)laser_vars.horizontal_scan_count;
        for(unsigned i = 0; i < laser_vars.horizontal_scan_count; i++)
        {
            laser_vars.output_scan.horizontal_scans[(laser_vars.horizontal_scan_count-1) - i].time = laser_vars.output_scan.time - base::Time::fromMicroseconds((int64_t)(time_between_scans_in_microseconds * i));

            // change horizontal angle direction. this is for standardization
            laser_vars.output_scan.horizontal_scans[i].horizontal_angle = laser_vars.output_scan.horizontal_scans[i].horizontal_angle * -1.0;
        }

        // write sample to output port
        if(laser_vars.head_pos == UpperHead)
            _laser_scans.write(laser_vars.output_scan);
        else
            _laser_scans_lower_head.write(laser_vars.output_scan);
        laser_vars.horizontal_scan_count = 0;
    }

    // increase array size of horizontal scans if needed
    if(laser_vars.horizontal_scan_count >= laser_vars.output_scan.horizontal_scans.size())
        laser_vars.output_scan.horizontal_scans.resize(laser_vars.horizontal_scan_count + 1);

    // add the new vertical scan to the output data
    laserdriver.convertToVerticalMultilevelScan(horizontal_scan, laser_vars.output_scan.horizontal_scans[laser_vars.horizontal_scan_count]);
    laser_vars.horizontal_scan_count++;
    laser_vars.last_sample_time = base::Time::now();
}

void LaserScanner::handleHorizontalScan(const velodyne_fire16_t& horizontal_scan, LaserScanner::LaserHeadVariables& laser_vars, int lower_upper_shot)
{   
    base::Angle scan_angle;
    if (lower_upper_shot == 1)
    {
        scan_angle = base::Angle::fromDeg(((double)horizontal_scan.rotational_pos) * 0.01);
    }
    else
    {
        scan_angle = base::Angle::fromDeg(((double)horizontal_scan.rotational_pos) * 0.01 + 0.2); //TODO: this assumes a rotational speed of 10 Hz, however it should be interpolated between the shot before and after.
    }

    if(isScanComplete(scan_angle, laser_vars))
    {

        base::Time current_time_stamp = base::Time::now();
        // resize the array of all scans to the actual count
        laser_vars.output_scan.horizontal_scans.resize(laser_vars.horizontal_scan_count);

        // interpolate time between the actual and the last sample for all vertical scans
        base::Time last_output_time = laser_vars.output_scan.time;
        laser_vars.output_scan.time = laser_vars.timestamp_estimator->update(laser_vars.last_sample_time);

        if(last_output_time.microseconds == 0 || last_output_time > laser_vars.output_scan.time)
            last_output_time = laser_vars.output_scan.time;

        double time_between_scans_in_microseconds = ((double)(laser_vars.output_scan.time - last_output_time).microseconds) / (double)laser_vars.horizontal_scan_count;
        for(unsigned i = 0; i < laser_vars.horizontal_scan_count; i++)
        {
            laser_vars.output_scan.horizontal_scans[(laser_vars.horizontal_scan_count-1) - i].time = laser_vars.output_scan.time - base::Time::fromMicroseconds((int64_t)(time_between_scans_in_microseconds * i));

            // change horizontal angle direction. this is for standardization
            laser_vars.output_scan.horizontal_scans[i].horizontal_angle = laser_vars.output_scan.horizontal_scans[i].horizontal_angle * -1.0;
        }

        // write sample to output port
        if(laser_vars.head_pos == UpperHead){

            _laser_scans.write(laser_vars.output_scan);

            int azimuth_count = 1808;//(int) laser_vars.horizontal_scan_count;

            //interpolated velodyne images are for easier viewing and possible use in algorithms
            char ir_interpImage[azimuth_count*150*2]; //note that the values are exactly 100 times larger than their corresponding raw measurements, so that no division is done. Divide by 100 to get a corresponding number between 0 - 255.
            char range_interpImage[azimuth_count*150*2]; //note that the values are exactly 10 times larger than their corresponding raw measurements, so that no division is done. Divide by 10 to get a corresponding number between 0 and 65535.
            char azimuth_interpImage[azimuth_count*150*2]; //note that the values are exactly 10 times larger than their corresponding raw measurements, so that no division is done. Divide by 10 to get a corresponding number between 0 and 65535.

            char irImage[azimuth_count*16];
            char rangeImage[azimuth_count*16*2];
            char azimuthImage[azimuth_count*16*2];

            uint8_t range_holder[2];
            uint8_t range_interp_holder[4];
            uint8_t azimuth_holder[2];
            uint16_t ir_holder[2];

            uint16_t range16;
            uint32_t range32;
            uint16_t ir16;
            uint16_t azimuth16;
            double azimuth_degree;
            uint16_t interp_ir_first;
            uint16_t interp_ir_second;
            uint32_t interp_range_first;
            uint32_t interp_range_second;

            if(!ir_frame){
                ir_frame = new base::samples::frame::Frame(azimuth_count,16,8,base::samples::frame::MODE_GRAYSCALE);
            }

            if(!ir_interp_frame){
                ir_interp_frame = new base::samples::frame::Frame(azimuth_count,150,16,base::samples::frame::MODE_GRAYSCALE);
            }

            if(!range_frame){
                range_frame = new base::samples::frame::Frame(azimuth_count,16,16,base::samples::frame::MODE_GRAYSCALE);
            }

            if(!range_interp_frame){
                range_interp_frame = new base::samples::frame::Frame(azimuth_count,150,16,base::samples::frame::MODE_GRAYSCALE);
            }

            if(!azimuth_frame){
                azimuth_frame = new base::samples::frame::Frame(azimuth_count,16,16,base::samples::frame::MODE_GRAYSCALE);
            }

            //handle interpolated images here.
            for(int j = 0; j<VELODYNE_NUM_LASERS-1; j++){
                for(int i = 0; i<azimuth_count; i++){
                    for(int k = 0; k<10; k++){

                        interp_ir_first = (uint16_t) laser_vars.output_scan.horizontal_scans[i].vertical_scans[j].remission;
                        interp_ir_first = interp_ir_first*(100-k*10);
                        interp_ir_second = (uint16_t) laser_vars.output_scan.horizontal_scans[i].vertical_scans[j+1].remission;
                        interp_ir_second = interp_ir_second*(k*10);
                        ir16 = interp_ir_first + interp_ir_second;

                        ir_holder[0] = ir16 & 0x00ff;
                        ir_holder[1] = (ir16 >> 8);
                        ir_interpImage[0 + j*azimuth_count*10*2 + i*2 + k*azimuth_count*2] = ir_holder[0];
                        ir_interpImage[1 + j*azimuth_count*10*2 + i*2 + k*azimuth_count*2] = ir_holder[1];

                        interp_range_first = (uint32_t)laser_vars.output_scan.horizontal_scans[i].vertical_scans[j].range;
                        interp_range_first = interp_range_first*(1000-k*100);
                        interp_range_second = (uint32_t)laser_vars.output_scan.horizontal_scans[i].vertical_scans[j+1].range;
                        interp_range_second = interp_range_second*(k*100);
                        range32 = interp_range_first + interp_range_second;

                        range_interp_holder[0] = (uint8_t)(range32 & 0x000000ff);
                        range_interp_holder[1] = (uint8_t)((range32 >> 8) & 0x000000ff);					
                        range_interp_holder[2] = (uint8_t)((range32 >> 16) & 0x000000ff);
                        range_interp_holder[3] = (uint8_t)((range32 >> 24) & 0x000000ff);
                        range_interpImage[0 + j*azimuth_count*10*2 + k*azimuth_count*2 + i*2] = range_interp_holder[3];
                        range_interpImage[1 + j*azimuth_count*10*2 + k*azimuth_count*2 + i*2] = range_interp_holder[2];

                        /*azimuth16 = (uint16_t)(laser_vars.output_scan.horizontal_scans[i].horizontal_angle.getDeg());

                          azimuth_holder[0] = azimuth16 & 0x00ff;
                          azimuth_holder[1] = (azimuth16 >> 8);
                          azimuthImage[0 + j*azimuth_count*10*2 + k*azimuth_count*2 + i*2] = azimuth_holder[0];
                          azimuthImage[1 + j*azimuth_count*10*2 + k*azimuth_count*2 + i*2] = azimuth_holder[1];*/
                    }
                }
            }

            //handle raw images here
            for(int j = 0; j<VELODYNE_NUM_LASERS; j++){
                for(int i = 0; i<azimuth_count; i++){
                    irImage[j*azimuth_count + i] = (char)laser_vars.output_scan.horizontal_scans[i].vertical_scans[j].remission;
                    range16 = laser_vars.output_scan.horizontal_scans[i].vertical_scans[j].range;

                    range_holder[0] = range16 & 0x00ff;
                    range_holder[1] = (range16 >> 8);
                    rangeImage[0 + j*azimuth_count*2 + i*2] = range_holder[0];
                    rangeImage[1 + j*azimuth_count*2 + i*2] = range_holder[1];

                    azimuth_degree = laser_vars.output_scan.horizontal_scans[i].horizontal_angle.getDeg();
                    if (azimuth_degree < 0){
                        azimuth16 = (uint16_t)((azimuth_degree+360)*100);
                    }else{
                        azimuth16 = (uint16_t)((azimuth_degree)*100);
                    }
                    //azimuth16 = (uint16_t)((laser_vars.output_scan.horizontal_scans[i].horizontal_angle.getDeg()+180)*100);
                    azimuth_holder[0] = azimuth16 & 0x00ff;
                    azimuth_holder[1] = (azimuth16 >> 8);
                    azimuthImage[0 + j*azimuth_count*2 + i*2] = (azimuth_holder[0]);
                    azimuthImage[1 + j*azimuth_count*2 + i*2] = (azimuth_holder[1]);

                }
            }

            // FIXME: Shouldn't these values use the same timestamp as the laser_scans?
            ir_interp_frame->setImage(ir_interpImage,azimuth_count*150*2);
            ir_interp_frame->time = current_time_stamp;
            ir_interp_frame_p.reset(ir_interp_frame);
            _ir_interp_frame.write(ir_interp_frame_p);

            ir_frame->setImage(irImage,azimuth_count*16);
            ir_frame->time = current_time_stamp;
            ir_frame_p.reset(ir_frame);
            _ir_frame.write(ir_frame_p);

            range_interp_frame->setImage(range_interpImage,azimuth_count*150*2);
            range_interp_frame->time = current_time_stamp;
            range_interp_frame_p.reset(range_interp_frame);
            _range_interp_frame.write(range_interp_frame_p);

            range_frame->setImage(rangeImage,azimuth_count*16*2);
            range_frame->time = current_time_stamp;
            range_frame_p.reset(range_frame);
            _range_frame.write(range_frame_p);

            azimuth_frame->setImage(azimuthImage,azimuth_count*16*2);
            azimuth_frame->time = current_time_stamp;
            azimuth_frame_p.reset(azimuth_frame);
            _azimuth_frame.write(azimuth_frame_p);

        }	
        else
        {
            _laser_scans_lower_head.write(laser_vars.output_scan);
        }
        laser_vars.horizontal_scan_count = 0;
    }

    // increase array size of horizontal scans if needed
    if(laser_vars.horizontal_scan_count >= laser_vars.output_scan.horizontal_scans.size())
        laser_vars.output_scan.horizontal_scans.resize(laser_vars.horizontal_scan_count + 1);
    // add the new vertical scan to the output data
    laserdriver.convertToVerticalMultilevelScan(horizontal_scan, laser_vars.output_scan.horizontal_scans[laser_vars.horizontal_scan_count],lower_upper_shot);
    laser_vars.horizontal_scan_count++;
    laser_vars.last_sample_time = base::Time::now();
}

void LaserScanner::addDummyData(const base::Angle& next_angle, LaserScanner::LaserHeadVariables& laser_vars)
{
    if(laser_vars.horizontal_scan_count > 1)
    {
        // get angular resolution of the first two scans
        base::Angle angular_resolution = laser_vars.output_scan.horizontal_scans[1].horizontal_angle - laser_vars.output_scan.horizontal_scans[0].horizontal_angle;
        if(angular_resolution.rad > 0.0)
        {
            base::Angle last_angle = laser_vars.output_scan.horizontal_scans[laser_vars.horizontal_scan_count-1].horizontal_angle;
            base::Angle new_angle = next_angle;
            // limit the new angle if the scan is almost complete 
            if(isScanComplete(next_angle, laser_vars))
                new_angle = laser_vars.output_scan.horizontal_scans[0].horizontal_angle;

            base::Angle angular_gap = new_angle - last_angle;
            if(angular_gap.rad > 0.0)
            {
                // fill the gap with dummy samples
                int missing_sample_count = ((int)(angular_gap.rad / angular_resolution.rad)) - 1;
                for(int i = 1; i <= missing_sample_count; i++)
                {
                    // add a dummy sample to the output scan
                    base::Angle missing_sample_angle = last_angle + i * angular_resolution;
                    velodyne_fire_t horizontal_scan;
                    createHorizontalDummy(missing_sample_angle, laser_vars.head_pos, horizontal_scan);
                    handleHorizontalScan(horizontal_scan, laser_vars);
                }
            }
        }
    }
}

bool LaserScanner::getFirstAngle(LaserHead head_pos, const velodyne_data_packet& new_scans, base::Angle& first_angle) const
{
    for(unsigned i = 0; i < VELODYNE_NUM_BLOCKS; i++)
    {
        if((head_pos == UpperHead && new_scans.shots[i].lower_upper == VELODYNE_UPPER_HEADER_BYTES) || 
                (head_pos == LowerHead && new_scans.shots[i].lower_upper == VELODYNE_LOWER_HEADER_BYTES))
        {
            first_angle = base::Angle::fromDeg(((double)new_scans.shots[i].rotational_pos) * 0.01);
            return true;
        }
    }
    return false;
}

bool LaserScanner::getFirstAngle(LaserHead head_pos, const velodyne_data_packet16& new_scans, base::Angle& first_angle) const
{
    for(unsigned i = 0; i < VELODYNE_NUM_BLOCKS; i++)
    {
        if((head_pos == UpperHead && new_scans.shots[i].lower_upper == VELODYNE_UPPER_HEADER_BYTES) || 
                (head_pos == LowerHead && new_scans.shots[i].lower_upper == VELODYNE_LOWER_HEADER_BYTES))
        {
            first_angle = base::Angle::fromDeg(((double)new_scans.shots[i].rotational_pos) * 0.01);
            return true;
        }
    }
    return false;
}

void LaserScanner::createHorizontalDummy(const base::Angle& angle, LaserScanner::LaserHead head_pos, velodyne_fire_t& horizontal_scan) const
{
    // set all 32 vertical scans to out of range
    for(unsigned i = 0; i < VELODYNE_NUM_LASERS; i++)
    {
        horizontal_scan.lasers[i].distance = 0;
        horizontal_scan.lasers[i].intensity = 0;
    }
    // set head position
    if(head_pos == UpperHead)
        horizontal_scan.lower_upper = VELODYNE_UPPER_HEADER_BYTES;
    else
        horizontal_scan.lower_upper = VELODYNE_LOWER_HEADER_BYTES;
    // set horizontal angle
    double angle_deg = angle.getDeg();
    if(angle_deg < 0.0)
        angle_deg += 360.0;
    horizontal_scan.rotational_pos = (uint16_t)(angle_deg * 100.0);
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.




bool LaserScanner::configureHook()
{
    if (! RTT::TaskContext::configureHook())
        return false;

    upper_head.timestamp_estimator = new aggregator::TimestampEstimator(base::Time::fromSeconds(20), base::Time::fromMilliseconds(100));
    lower_head.timestamp_estimator = new aggregator::TimestampEstimator(base::Time::fromSeconds(20), base::Time::fromMilliseconds(100));

    return true;
}

bool LaserScanner::startHook()
{

    if (! RTT::TaskContext::startHook())
        return false;

    // setup udp server
    laserdriver.openUDP("", VELODYNE_DATA_UDP_PORT);

    // trigger the update hook on fd activity
    RTT::extras::FileDescriptorActivity* fd_activity =
        getActivity<RTT::extras::FileDescriptorActivity>();
    if (fd_activity)
    {
        fd_activity->watch(laserdriver.getFileDescriptor());
    }
    else
    {
        RTT::log(RTT::Error) << TaskContext::getName() << ": "
            << "Error: the task needs to be fd driven." << RTT::endlog();
        return false;
    }

    last_state = PRE_OPERATIONAL;
    last_packet_period = 1000000;
    last_gps_timestamp = 0;
    gps_timestamp_tolerance = 100;
    unsigned scans_per_turn = 20000; // upper guess
    unsigned min_range = _min_range.get() * 1000; // m to mm
    unsigned max_range = _max_range.get() * 1000; // m to mm

    // initialize time reference
    lidar_time_ref = base::Time::now().toMicroseconds();

    // initiate head variables
    upper_head.timestamp_estimator->reset();
    lower_head.timestamp_estimator->reset();
    upper_head.horizontal_scan_count = 0;
    lower_head.horizontal_scan_count = 0;
    upper_head.output_scan.horizontal_scans.reserve(scans_per_turn);
    lower_head.output_scan.horizontal_scans.reserve(scans_per_turn);
    upper_head.output_scan.time.microseconds = 0;
    lower_head.output_scan.time.microseconds = 0;
    upper_head.output_scan.min_range = min_range;
    upper_head.output_scan.max_range = max_range;
    lower_head.output_scan.min_range = min_range;
    lower_head.output_scan.max_range = max_range;
    upper_head.head_pos = UpperHead;
    lower_head.head_pos = LowerHead;

    return true;
}



void LaserScanner::updateHook()
{
    RTT::TaskContext::updateHook();

    States actual_state = RUNNING;

    base::Time timeout = base::Time::fromMilliseconds(_timeout.get());


    int size = 0;
    uint32_t gps_timestamp;
    try 
    {
        if (VELODYNE_NUM_LASERS == 16)
        {
            size = laserdriver.readPacket((uint8_t*)&buffer16, VELODYNE_DATA_MSG_BUFFER_SIZE, timeout, timeout);
        }
        else
        {
            size = laserdriver.readPacket((uint8_t*)&buffer32or64, VELODYNE_DATA_MSG_BUFFER_SIZE, timeout, timeout);
        }
    }
    catch (std::runtime_error e)
    {
        RTT::log(RTT::Error) << TaskContext::getName() << ": " << e.what() << RTT::endlog();
        actual_state = IO_TIMEOUT;
    } 
    if(size == (int)VELODYNE_DATA_MSG_BUFFER_SIZE)
    {
        // check packet timestamp
        if (VELODYNE_NUM_LASERS == 16)
        {
            gps_timestamp = buffer16.gps_timestamp;
        }
        else
        {
            gps_timestamp = buffer32or64.gps_timestamp;
        }

        // Update the timestamp estimator(s) using the reference from LiDAR
        // If we already have got a last timestamp we can calc a difference of the 32 Bit timestamps and accumulate it on top of a 64 Bit storage
        if (last_gps_timestamp > 0)
        {
            // Accumulate the 32 Bit timestamps in a 64 Bit storage
            // According to the datasheet the 32 Bit timestamps are clamped at hour level, so we have to use modulo here
            uint32_t diff = ((gps_timestamp - last_gps_timestamp) % (3600 * 1000000));
            lidar_time_ref = lidar_time_ref + diff;
            upper_head.timestamp_estimator->updateReference( base::Time::fromMicroseconds(lidar_time_ref) );
            lower_head.timestamp_estimator->updateReference( base::Time::fromMicroseconds(lidar_time_ref) );
        }

        if(last_gps_timestamp > 0 && last_gps_timestamp < gps_timestamp)
        {
            if(last_packet_period + gps_timestamp_tolerance < gps_timestamp - last_gps_timestamp)
            {
                // handle packet lost
                RTT::log(RTT::Info) << TaskContext::getName() << ": "
                    << "Lost at least one data packet. Filling up the hole in the output scan with dummy data." << RTT::endlog();

                base::Angle first_upper_angle, first_lower_angle;
                if (VELODYNE_NUM_LASERS == 16)
                {
                    if(getFirstAngle(UpperHead, buffer16, first_upper_angle))
                    {
                        addDummyData(first_upper_angle, upper_head);
                    }
                    if(getFirstAngle(LowerHead, buffer16, first_lower_angle))
                    {
                        addDummyData(first_lower_angle, lower_head);
                    }
                }
                else
                {
                    if(getFirstAngle(UpperHead, buffer32or64, first_upper_angle))
                    {
                        addDummyData(first_upper_angle, upper_head);
                    }
                    if(getFirstAngle(LowerHead, buffer32or64, first_lower_angle))
                    {
                        addDummyData(first_lower_angle, lower_head);
                    }
                }
            }
            last_packet_period = gps_timestamp - last_gps_timestamp;
        }
        last_gps_timestamp = gps_timestamp;

        uint16_t packet_lower_upper;

        for(unsigned i = 0; i < VELODYNE_NUM_BLOCKS; i++)
        {
            if (VELODYNE_NUM_LASERS == 16)
            {
                packet_lower_upper = buffer16.shots[i].lower_upper;
            }
            else
            {
                packet_lower_upper = buffer32or64.shots[i].lower_upper;
            }

            if(packet_lower_upper == VELODYNE_UPPER_HEADER_BYTES)
            {
                // check if VLP 16
                if (VELODYNE_NUM_LASERS == 16)
                {
                    //handle lasers of VLP 16, where each datablock represents 2 shots
                    handleHorizontalScan(buffer16.shots[i], upper_head,1);
                    handleHorizontalScan(buffer16.shots[i], upper_head,2);
                }
                else
                {
                    std::cout << "You shouldn't be here, this is not definitively a HDL 32" << std::endl;
                    // handle lasers of HDL 32 and 64, where each datablock represents 1 shot
                    handleHorizontalScan(buffer32or64.shots[i], upper_head);
                }
            }
            else
            {
                std::cout << "You shouldn't be here, this is not a HDL 64" << std::endl;
                // handle upper laser, only needed for HDL 64
                handleHorizontalScan(buffer32or64.shots[i], lower_head);
            }
        }
    }
    else
    {
        RTT::log(RTT::Error) << TaskContext::getName() << ": "
            << "Received an unknown packet of size " << size 
            << ". Velodyne data message should be of size " 
            << VELODYNE_DATA_MSG_BUFFER_SIZE << RTT::endlog();
        actual_state = IO_ERROR;
    }

    // write state if it has changed
    if(last_state != actual_state)
    {
        last_state = actual_state;
        state(actual_state);
    }

}



void LaserScanner::errorHook()
{
    RTT::TaskContext::errorHook();
}



void LaserScanner::stopHook()
{
    RTT::TaskContext::stopHook();

    laserdriver.close();
}



void LaserScanner::cleanupHook()
{
    RTT::TaskContext::cleanupHook();

    delete upper_head.timestamp_estimator;
    delete lower_head.timestamp_estimator;
}

