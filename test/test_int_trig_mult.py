from Lima import Core, Eiger
import time
import sys
import getopt
import numpy as np
from collections import namedtuple
from contextlib import contextmanager
from threading import Condition, Event

AcqPars = namedtuple('AcqPars',
                     ['trig_mode', 'nb_frames', 'expo_time', 'lat_time'])

class TestIntTrig:

  class Cb(Core.CtControl.ImageStatusCallback):
    def __init__(self, frame_ts, end, acq_pars):
      super().__init__()
      self.last_acquired = -1
      self.frame_ts = frame_ts
      self.end = end
      self.acq_pars = acq_pars

    def imageStatusChanged(self, status):
      if status.LastImageAcquired == self.last_acquired:
        return

      self.frame_ts.append(time.time())
      self.last_acquired = status.LastImageAcquired

      if self.last_acquired == self.acq_pars.nb_frames - 1:
        self.end.set()

  def __init__(self, hw_inter):
    self.hw_inter = hw_inter
    self.ct = Core.CtControl(self.hw_inter)
    self.acq = self.ct.acquisition()
    self.sync = self.hw_inter.getHwCtrlObj(Core.HwCap.Sync)

  def run(self, acq_pars):
    if acq_pars.trig_mode == Core.IntTrig:
      self.runIntTrig(acq_pars)
    else:
      self.runIntTrigMult(acq_pars)

  def prepare(self, acq_pars):
    self.acq.setTriggerMode(acq_pars.trig_mode)
    self.acq.setAcqNbFrames(acq_pars.nb_frames)
    self.acq.setAcqExpoTime(acq_pars.expo_time)
    self.acq.setLatencyTime(acq_pars.lat_time)

    self.ct.prepareAcq()

    ranges = self.sync.getValidRanges()
    print("Valid ranges: ")
    print(f"  min_exp_time={ranges.min_exp_time:.7f},"
          f"  max_exp_time={ranges.max_exp_time:.7f}")
    print(f"  min_lat_time={ranges.min_lat_time:.7f},"
          f"  max_lat_time={ranges.max_lat_time:.7f}")

  def runIntTrig(self, acq_pars):
    frame_ts = []
    end = Event()
    cb = self.Cb(frame_ts, end, acq_pars)
    cb.setRatePolicy(cb.RateAllFrames)
    self.ct.registerImageStatusCallback(cb)

    self.prepare(acq_pars)

    print("Starting acquisition ...")
    t0 = time.time()
    self.ct.startAcq()
    end.wait()
    trigger_delay = time.time() - t0
    while self.ct.getStatus().AcquisitionStatus != Core.AcqReady:
      pass
    acq_delay = time.time() - t0

    if len(frame_ts) != acq_pars.nb_frames:
      print(f"Missing triggers: {len(frame_ts)}, expected {acq_pars.nb_frames}")
    bin_size = 5e-3
    bins = self.splitTimeBins(frame_ts, acq_pars, bin_size)
    print(f"Frame delay bins: {len(bins)} ({bin_size} size)")
    for (ave, std), b in sorted(bins):
      print(f"  n={len(b):02d}, ave={ave:.6f}, std={std:.6f}")
    first_trigger_delay = frame_ts[0] - t0
    point_time = acq_pars.expo_time + acq_pars.lat_time
    first_trigger_overhead = first_trigger_delay - point_time
    acq_time = acq_pars.nb_frames * point_time
    overhead = trigger_delay - acq_time
    print(f"First trigger delay: {first_trigger_delay:.3f} "
          f"({point_time:.3f} + {first_trigger_overhead:.3f})")
    print(f"Trigger delay: {trigger_delay:.3f} "
          f"({acq_time:.3f} + {overhead:.3f})")
    print(f"Acquisition finished after {acq_delay:.3f} sec "
          f"({trigger_delay:.3f} + {acq_delay - trigger_delay:.3f})")

  def splitTimeBins(self, ts, acq_pars, bin_size):
    deltas = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
    bins = []
    for t in deltas:
      for b in bins:
        if abs(t - np.average(b)) <= bin_size:
          b.append(t)
          break
      else:
        bins.append([t])
    return [((np.average(b), np.std(b)), b) for b in bins]

  def runIntTrigMult(self, acq_pars):
    self.prepare(acq_pars)

    print("Starting acquisition ...")
    trigger_delay = []
    for i in range(nb_frames):
      print(f"Sending start #{i}")
      self.ct.startAcq()
      t0 = time.time()
      while self.hw_inter.getStatus().det != Core.DetIdle:
        pass
      trigger_delay.append(time.time() - t0)

    t0 = time.time()
    while self.ct.getStatus().AcquisitionStatus != Core.AcqReady:
      pass
    delay = time.time() - t0

    last_delay = trigger_delay.pop(-1)
    ave_trigger_delay = sum(trigger_delay) / len(trigger_delay)
    overhead = ave_trigger_delay - expo_time
    print(f"Average trigger_delay={ave_trigger_delay:.3f} "
          f"({expo_time:.3f} + {overhead:.3f})")

    delay += last_delay - ave_trigger_delay
    print(f"Acquisition finished {delay:.3f} sec after trigger sequence")


if __name__ == '__main__':
  verbose = False
  nb_frames = 10
  expo_time = 0.01
  lat_time = 2e-3
  trig_mode = 'IntTrigMult'

  opts, args = getopt.getopt(sys.argv[1:], 'vn:e:l:t:')
  for opt, val in opts:
    if opt == '-v':
      verbose = True
    if opt == '-n':
      nb_frames = int(val)
    if opt == '-e':
      expo_time = float(val)
    if opt == '-l':
      lat_time = float(val)
    if opt == '-t':
      trig_mode = val

  host = args[0]

  acq_pars = AcqPars(getattr(Core, trig_mode), nb_frames, expo_time, lat_time)

  if verbose:
    db = Core.DebParams
    db.setTypeFlagsNameList(['Funct','Trace','Param','Return','Warning','Fatal'])
    #db.setTypeFlagsNameList(['Funct', 'Trace','Fatal'])
    #db.setModuleFlagsNameList(['Camera'])

  cam = Eiger.Camera(host)
  hw_inter = Eiger.Interface(cam)

  test = TestIntTrig(hw_inter)
  test.run(acq_pars)

  stream_info = hw_inter.getLastStreamInfo()
  print(f"Stream info: encoding={stream_info.encoding}, "
        f"packed_size={stream_info.packed_size}")
  stream_stats = hw_inter.latchStreamStatistics()
  print(f"Stream stats: ave_speed={stream_stats.ave_speed() / 1e6:.3f} MB/s")
