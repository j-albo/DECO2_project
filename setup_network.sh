#!/bin/bash

echo "🌐 Configurando red para Livox Mid-360"

# Tu interfaz Ethernet detectada
ETH_INTERFACE="enxe8ea6a6fa664"

echo "📡 Configurando interfaz: $ETH_INTERFACE"

# Configurar IP estática para comunicación con Mid-360
echo "🔧 Configurando IP 192.168.1.5 en $ETH_INTERFACE"
sudo ip addr add 192.168.1.5/24 dev $ETH_INTERFACE 2>/dev/null || true
sudo ip link set $ETH_INTERFACE up

# Verificar configuración
echo "✅ Configuración actual:"
ip addr show $ETH_INTERFACE

echo ""
echo "🧪 Estado de la interfaz:"
nmcli device status | grep $ETH_INTERFACE

echo ""
echo "📋 Próximos pasos:"
echo "   1. Conecta el cable Ethernet del Mid-360 al adaptador"
echo "   2. Enciende el Mid-360" 
echo "   3. Prueba conectividad: ping 192.168.1.12"
EOF
