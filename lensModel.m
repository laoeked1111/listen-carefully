%% Define variables

f0 = 5e3; % [Hz]
cs_air = 343; % [m/s]
wavelength = cs_air / f0; % [m]

og_L = 60e-3; % [m]
og_w = 240e-3 / 8; % [m]

% scale the existing lens
L = wavelength; % [m]
w = og_w *L / og_L; % [m]

wall_thickness = 0.1 * w; % [m]

N = 8;


%% Define computational grid

Nx = 512 - 40;
Ny = 256 - 40;

xLength = 12 * wavelength; % [m]
yLength = (N * w + (N+1) * wall_thickness) * 1.2; % [m]

dx = xLength / Nx; % [m]
dy = yLength / Ny; % [m]

Nperiods = 16;
T = Nperiods / f0;
Nt = 1024;

kgrid = kWaveGrid(Nx, dx, Ny, dy);

%% Define medium

cs_plastic = 2591; % [m/s]
density_air = 1; % [kg/m^3]
density_plastic = 1178; % [kg/m^3]

lens_mask = zeros(Nx, Ny);

w_pixels = round(w / dy);
L_pixels = round(L / dx);
wall_pixels = round(wall_thickness / dy);

% teeth calculations
cells = zeros(N);

dist = linspace(-N/2 + 0.5, N/2 - 0.5, N);
for i = 1:8
  for j = 1:8
    cells(i, j) = sqrt(dist(i)^2 + dist(j)^2);
  end
end

% phi(r) = phi0 - A^2r^2
% t=0.1w to 0.8w
A = sqrt(0.35 * w * 2 * pi / wavelength / 24);
phi0 = 0.55*w*2*pi/wavelength + 24.5 * A^2;

phases = phi0 - A^2 * cells .^ 2;
t = phases * wavelength / pi - w; % teeth length
t_pixels = t / dy; % teeth length in pixels

t_pixels_line = t_pixels(4, :);

% create cells
for i = 1:N+1

  % walls
  wall_start_y = round(1/12 * Ny) + (i-1) * (wall_pixels + w_pixels);
  wall_start_x = Nx / 8;
  lens_mask(wall_start_x:(wall_start_x + L_pixels), wall_start_y:(wall_start_y + wall_pixels)) = 1;

  if i == N+1
    break
  end

  % teeth
  if i <= N/2
    t1_start = round(wall_start_x + L_pixels / 4);
    t2_start = round(wall_start_x + L_pixels * 3 / 4);
  else
    t2_start = round(wall_start_x + L_pixels / 4);
    t1_start = round(wall_start_x + L_pixels * 3 / 4);
  end

  this_t_pixels = round(t_pixels_line(i));
  next_wall_start_y = wall_start_y + (wall_pixels + w_pixels);

  lens_mask(t1_start:(t1_start + wall_pixels), (wall_start_y + wall_pixels):(wall_start_y + wall_pixels + this_t_pixels)) = 1;
  lens_mask(t2_start:(t2_start + wall_pixels), (next_wall_start_y - this_t_pixels):(next_wall_start_y)) = 1;
end

figure(1);
imagesc(lens_mask)
xlabel("Y axis")
ylabel("X axis")
title("Lens mask")
axis square
colorbar

% define medium
medium.sound_speed = cs_air * ones(Nx, Ny);
medium.density = density_air * ones(Nx, Ny);
medium.sound_speed(lens_mask == 1) = cs_plastic;
medium.density(lens_mask == 1) = density_plastic;

figure(2)
imagesc(medium.sound_speed)
xlabel("Y axis")
ylabel("X axis")
title("Sound speed")
axis square
colorbar

figure(3)
imagesc(medium.density)
xlabel("Y axis")
ylabel("X axis")
title("Density")
axis square
colorbar


cfl = 0.1;
kgrid.makeTime(medium.sound_speed, cfl, T);


%% Define source

source.p_mask = zeros(Nx, Ny);
source.p_mask(1, round(Ny/12):round(Ny*11/12)) = 1;
source.p = sin(2 * pi * f0 * kgrid.t_array);

%% Define sensor

sensor.mask = ones(Nx, Ny);
sensor.record = {'p', 'p_max'};

%% Run sim

sensor_data = kspaceFirstOrder2D(kgrid, medium, source, sensor,...
  'DisplayMask', zeros(Nx, Ny),...
  'PMLInside', false,...
  'PlotSim', true,...
  'PlotLayout', true);

%%

reshaped_p = reshape(sensor_data.p, Nx, Ny, []);
reshaped_max_p = reshape(sensor_data.p_max, Nx, Ny);

save("lensModelResults1", "reshaped_max_p");

%%

figure(4); clf;
imagesc(reshaped_max_p)
caxis([0 0.5])
pbaspect([yLength xLength 1])
xlabel("Y axis")
ylabel("X axis")
title("Max pressure during simulation")
colorbar

