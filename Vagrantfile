# -*- mode: ruby -*-
# vi: set ft=ruby :

# Testing an ArduPilot VM:
# sim_vehicle.py --map --console # in the starting directory should start a Copter simulation
# sim_vehicle.py --debug --gdb
# sim_vehicle.py --valgrind
# time (cd /vagrant && ./waf configure --board=fmuv2 && ./waf build --target=bin/ardusub) # ~9 minutes
# time (cd /vagrant && ./waf configure --board=fmuv3 && ./waf build --target=bin/ardusub) # ~ minutes (after building fmuv2)
# time (cd /vagrant && ./waf configure --board=navio2 && ./waf build --target=bin/arduplane)
# time (cd /vagrant && ./Tools/autotest/sim_vehicle.py --map --console -v ArduPlane -f jsbsim) # should test JSBSim
# time (cd /vagrant && ./Tools/autotest/autotest.py build.APMrover2 drive.APMrover2)

# Vagrantfile API/syntax version. Don't touch unless you know what you're doing!
VAGRANTFILE_API_VERSION = "2"

Vagrant.configure(VAGRANTFILE_API_VERSION) do |config|
  config.ssh.forward_x11 = true

  # Provider-specific configuration so you can fine-tune various
  # backing providers for Vagrant. These expose provider-specific options.
  # Example for VirtualBox:
  #
  config.vm.provider "virtualbox" do |vb|
      # Don't boot with headless mode
      #   vb.gui = true
      #
      #   # Use VBoxManage to customize the VM. For example to change memory:
      vb.customize ["modifyvm", :id, "--memory", "3192"]
      vb.customize ["modifyvm", :id, "--ioapic", "on"]
      vb.customize ["modifyvm", :id, "--cpus", "2"]
      # Make some effort to avoid clock skew
      vb.customize ["guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-threshold", "5000"]
      vb.customize ["guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-start"]
      vb.customize ["guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-on-restore", "1"]
  end

  # If you are on windows then you must use a version of git >= 1.8.x
  # to update the submodules in order to build. Older versions of git
  # use absolute paths for submodules which confuses things.

  # removing this line causes "A box must be specified." error
  # and this is the default box that will be booted if no name is specified
  config.vm.box = "ubuntu/bionic64"

  # 18.04 LTS
  config.vm.define "bionic64", primary: true do |bionic64|
    config.vm.box = "ubuntu/bionic64"
    config.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"
    config.vm.provider "virtualbox" do |vb|
      vb.name = "Harris ArduPilot (bionic64)"
    end
  end

end

