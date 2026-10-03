To back up sd card:
Find disk name with `diskutil list`
Run `sudo dd if=/dev/rdisk4 of=/Volumes/<folder>/pi_backup.img bs=1m status=progress` 
Pishrink https://github.com/Drewsif/pishrink

On Pi:
```bash
# Initial setup and upgrading packages
sudo apt update && sudo apt full-upgrade -y
sudo apt autoremove -y
sudo apt clean
sudo reboot

# Install build tools for Hailo
sudo apt install build-essential dkms -y

# Optional: Install neovim + lazyvim
sudo apt install -y git cmake build-essential gettext curl ripgrep fd-find fzf
git clone https://github.com/neovim/neovim.git
cd neovim
git checkout stable
make CMAKE_BUILD_TYPE=RelWithDebInfo
cd build && cpack -G DEB
sudo dpkg -i nvim-linux-arm64.deb
cd ~
sudo rm -rf neovim

git clone https://github.com/LazyVim/starter ~/.config/nvim
nvim

# Install zsh and oh-my-zsh
sudo apt install zsh -y
zsh
```
```zsh
sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"
git clone --depth=1 https://github.com/romkatv/powerlevel10k.git "${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k"
nvim .zshrc # Change ZSH_THEME to ZSH_THEME="powerlevel10k/powerlevel10k"
source ~/.zshrc # Go through the customisation of powerlevel10k
```

Now some nice addons for oh-my-zsh:
```zsh
git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting
```

and add `zsh-autosuggestions` and `zsh-syntax-highlighting` to 'plugins' within .zshrc

Make sure that raspi-config is set up correctly, including enabling i2c and spi

Add this to boot/firmware/config.txt:
```bash
#fan config
dtparam=fan_temp0=45000,fan_temp0_hyst=2000,fan_temp0_speed=50
dtparam=fan_temp1=60000,fan_temp1_hyst=3000,fan_temp1_speed=80
dtparam=fan_temp2=70000,fan_temp2_hyst=4000,fan_temp1_speed=120
dtparam=fan_temp3=80000,fan_temp3_hyst=5000,fan_temp1_speed=255
```
Install docker:
```zsh
sudo apt remove $(dpkg --get-selections docker.io docker-compose docker-doc docker-buildx podman-docker containerd runc | cut -f1)

# Add Docker's official GPG key:
sudo apt update
sudo apt install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/debian
Suites: $(. /etc/os-release && echo "$VERSION_CODENAME")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt update

sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y
sudo groupadd docker
sudo usermod -aG docker $USER
```

Clone the repository:
```zsh
git clone https://github.com/Wet-Lettuce-Robocup/robocup-ros.git
git submodule update --init --recursive
```
(Make sure that the right branches are checked out)

Install final dependencies:
```zsh
cd device_tree
./install.sh
cd ..

sudo dpkg --install hailo_dependencies/hailort-pcie-driver_5.3.0_all.deb
```
You may get an error from dkms due to outdated wrappers being used. This will be fixed in 5.4.0, but for now see https://github.com/hailo-ai/hailort-drivers/pull/52

Finally, run `docker compose up --build` to launch the container.