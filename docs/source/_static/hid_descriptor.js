// Interactive HID Descriptor JavaScript

document.addEventListener('DOMContentLoaded', function() {
    // Add smooth scrolling for anchor links
    document.querySelectorAll('a[href^="#"]').forEach(anchor => {
        anchor.addEventListener('click', function (e) {
            const target = document.querySelector(this.getAttribute('href'));
            if (target) {
                e.preventDefault();
                target.scrollIntoView({
                    behavior: 'smooth',
                    block: 'start'
                });
            }
        });
    });

    // Enhanced tooltip positioning for mobile
    const bytes = document.querySelectorAll('.hid-byte');
    bytes.forEach(byte => {
        byte.addEventListener('mouseenter', function() {
            const tooltip = this.querySelector('.hid-byte-tooltip');
            if (tooltip && window.innerWidth <= 768) {
                document.body.style.overflow = 'hidden';
            }
        });

        byte.addEventListener('mouseleave', function() {
            if (window.innerWidth <= 768) {
                document.body.style.overflow = '';
            }
        });
    });

    // Add copy functionality to code blocks
    document.querySelectorAll('pre code').forEach(block => {
        const button = document.createElement('button');
        button.className = 'copy-button';
        button.textContent = 'Copy';
        button.style.cssText = `
            position: absolute;
            top: 8px;
            right: 8px;
            padding: 4px 12px;
            background: #4a5568;
            color: white;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            font-size: 0.8em;
            opacity: 0;
            transition: opacity 0.2s;
        `;

        const pre = block.parentElement;
        pre.style.position = 'relative';

        pre.addEventListener('mouseenter', () => {
            button.style.opacity = '1';
        });

        pre.addEventListener('mouseleave', () => {
            button.style.opacity = '0';
        });

        button.addEventListener('click', () => {
            navigator.clipboard.writeText(block.textContent).then(() => {
                button.textContent = 'Copied!';
                setTimeout(() => {
                    button.textContent = 'Copy';
                }, 2000);
            });
        });

        pre.appendChild(button);
    });
});

// Helper function to create interactive descriptor
function createHIDDescriptor(bytes, title) {
    const container = document.createElement('div');
    container.className = 'hid-descriptor-container';

    const titleElem = document.createElement('div');
    titleElem.className = 'hid-descriptor-title';
    titleElem.textContent = title || 'HID Report Descriptor';
    container.appendChild(titleElem);

    const bytesContainer = document.createElement('div');
    bytesContainer.className = 'hid-bytes-container';

    bytes.forEach(byteData => {
        const byteElem = document.createElement('div');
        byteElem.className = `hid-byte ${byteData.type || ''}`;

        const value = document.createElement('span');
        value.className = 'hid-byte-value';
        value.textContent = byteData.value;
        byteElem.appendChild(value);

        if (byteData.label) {
            const label = document.createElement('span');
            label.className = 'hid-byte-label';
            label.textContent = byteData.label;
            byteElem.appendChild(label);
        }

        if (byteData.tooltip) {
            const tooltip = document.createElement('div');
            tooltip.className = 'hid-byte-tooltip';

            if (byteData.tooltip.title) {
                const tooltipTitle = document.createElement('span');
                tooltipTitle.className = 'hid-byte-tooltip-title';
                tooltipTitle.textContent = byteData.tooltip.title;
                tooltip.appendChild(tooltipTitle);
            }

            const tooltipDesc = document.createElement('span');
            tooltipDesc.className = 'hid-byte-tooltip-desc';
            tooltipDesc.textContent = byteData.tooltip.description;
            tooltip.appendChild(tooltipDesc);

            byteElem.appendChild(tooltip);
        }

        bytesContainer.appendChild(byteElem);
    });

    container.appendChild(bytesContainer);

    return container;
}

// Export for use in HTML
window.createHIDDescriptor = createHIDDescriptor;
