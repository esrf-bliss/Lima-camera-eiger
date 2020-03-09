from Lima  import Core,Eiger
import time
import sys
import getopt

class TestIntTrig:
  def __init__(self, hw_inter):
    self.hw_inter = hw_inter
    self.ct = Core.CtControl(self.hw_inter)
    self.acq = self.ct.acquisition()
    self.sync = self.hw_inter.getHwCtrlObj(Core.HwCap.Sync)

  def run(self, nb_frames, expo_time, lat_time):
    self.acq.setTriggerMode(Core.IntTrigMult)
    self.acq.setAcqNbFrames(nb_frames)
    self.acq.setAcqExpoTime(expo_time)
    self.acq.setLatencyTime(lat_time)
    self.ct.prepareAcq()

    ranges = self.sync.getValidRanges()
    print("Valid ranges: ")
    print(f"  min_exp_time={ranges.min_exp_time:.7f}, max_exp_time={ranges.max_exp_time:.7f}")
    print(f"  min_lat_time={ranges.min_lat_time:.7f}, max_lat_time={ranges.max_lat_time:.7f}")

    print("Starting acquisition ...")
    trigger_delay = []
    for i in range(nb_frames):
      print("Sending start #{}".format(i))
      self.ct.startAcq()
      t0 = time.time()
      while self.hw_inter.getStatus().det != Core.DetIdle:
        pass
      trigger_delay.append(time.time() - t0)
    print("Trigger sequence finished")

    last_delay = trigger_delay.pop(-1)
    ave_trigger_delay = sum(trigger_delay) / len(trigger_delay)
    print(f"Average trigger_delay={ave_trigger_delay:.3f}")

    t0 = time.time()
    while self.ct.getStatus().AcquisitionStatus != Core.AcqReady:
      pass
    delay = time.time() - t0
    delay += last_delay - ave_trigger_delay
    print(f"Acquisition finished {delay:.3f} sec after trigger sequence")

if __name__ == '__main__':
  verbose = False
  nb_frames = 10
  expo_time = 0.01
  lat_time = 2e-3

  opts, args = getopt.getopt(sys.argv[1:], 'vn:e:l:')
  for opt, val in opts:
    if opt == '-v':
      verbose = True
    if opt == '-n':
      nb_frames = int(val)
    if opt == '-e':
      expo_time = float(val)
    if opt == '-l':
      lat_time = float(val)

  host = args[0]

  if verbose:
    db=Core.DebParams
    db.setTypeFlagsNameList(['Funct','Trace','Param','Return','Warning','Fatal'])
    #db.setTypeFlagsNameList(['Funct', 'Trace','Fatal'])
    #db.setModuleFlagsNameList(['Camera'])

  cam = Eiger.Camera(host)
  hw_inter = Eiger.Interface(cam)

  test = TestIntTrig(hw_inter)
  test.run(nb_frames, expo_time, lat_time)

  stream_info = hw_inter.getLastStreamInfo()
  print(f"Stream info: encoding={stream_info.encoding}, packed_size={stream_info.packed_size}")
  stream_stats = hw_inter.latchStreamStatistics()
  print(f"Stream stats: ave_speed={stream_stats.ave_speed() / 1e6:.3f} MB/s")
