import numpy as np
import cv2
import imageio.v3 as iio
from scipy.ndimage import gaussian_filter
import OpenImageIO as oiio
import numpy as np



def bilateral_denoise(indirect, sigma_d=0.4, sigma_r=2):
    """
    Aplica el filtro bilateral mejorado (con estimación previa) a una imagen ruidosa.
    
    Args:
        indirect (np.ndarray): Imagen ruidosa (componente indirecta)
        sigma_d (float): Sigma espacial
        sigma_r (float): Sigma de rango (color)
        
    Returns:
        np.ndarray: Imagen denoised
    """
    # Paso 1: estimación inicial (f̃) usando un filtro gaussiano espacial
    f_tilde = gaussian_filter(indirect, sigma=sigma_d)

    # Paso 2: aplicar filtro bilateral usando f̃
    height, width, channels = indirect.shape
    output = np.zeros_like(indirect)

    radius = int(3 * sigma_d)
    window = 2 * radius + 1
    X, Y = np.meshgrid(np.arange(-radius, radius + 1), np.arange(-radius, radius + 1))
    spatial_weights = np.exp(-(X**2 + Y**2) / (2 * sigma_d**2)) # Si estás cerca físicamente 
    spatial_weights = spatial_weights[:, :, np.newaxis]

    for i in range(radius, height - radius):
        for j in range(radius, width - radius):
            region = indirect[i - radius:i + radius + 1, j - radius:j + radius + 1, :]
            region_f = f_tilde[i - radius:i + radius + 1, j - radius:j + radius + 1, :]

            center_est = f_tilde[i, j, :]
            color_diff = region - center_est
            range_weights = np.exp(-(color_diff**2) / (2 * sigma_r**2))

            # Producto punto (spatial * range) por canal
            weights = spatial_weights * range_weights # Si te pareces en color
            norm = np.sum(weights, axis=(0, 1))
            weighted_sum = np.sum(region * weights, axis=(0, 1))
            output[i, j, :] = weighted_sum / (norm + 1e-8)

    return output


def load_exr(path_file):
    img_input = oiio.ImageInput.open(path_file)
    spec = img_input.spec()
    data = img_input.read_image(format=oiio.FLOAT)
    img_input.close()

    imagen = np.array(data).reshape(spec.height, spec.width, spec.nchannels)
    print("Shape:", imagen.shape)
    print("Tipo:", imagen.dtype)

    return imagen, spec

def save_exr(output_path, imagen_to_save, spec):
    out_spec = oiio.ImageSpec(spec.width, spec.height, spec.nchannels, oiio.FLOAT)
    out_img = oiio.ImageOutput.create(output_path)
    out_img.open(output_path, out_spec)
    out_img.write_image(imagen_to_save.flatten())
    out_img.close()


def save_png(output_path, image):
    image_clamped = np.clip(image, 0.0, 1.0)
    image_uint8 = (image_clamped * 255).astype(np.uint8)
    iio.imwrite(output_path, image_uint8)

if __name__ == "__main__":
    # Load the image
    indirect_array, spec = load_exr("indirect_dof.exr")
    direct_array, spec2 = load_exr("direct_dof.exr")

    bilateral_denoised = bilateral_denoise(indirect_array, sigma_d=3.0, sigma_r=2)

    print("Denoised image shape:", bilateral_denoised.shape)
    print("DIrect image shape:", direct_array.shape)

    final_image = direct_array + bilateral_denoised

    save_exr("output_denoised.exr", bilateral_denoised, spec)
    save_exr("output_final.exr", final_image, spec)
    
    # save_png("output_denoised.png", bilateral_denoised)
    # save_png("output_final.png", final_image)