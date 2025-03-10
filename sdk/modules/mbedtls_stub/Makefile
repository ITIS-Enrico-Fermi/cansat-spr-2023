############################################################################
# modules/mbedtls_stub/Makefile
#
#   Copyright 2021,2022 Sony Semiconductor Solutions Corporation
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name of Sony Semiconductor Solutions Corporation nor
#    the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

-include $(SDKDIR)/modules/Make.defs

DELIM ?= $(strip /)
CXXEXT ?= .cpp

MODNAME = mbedtls_stub

# Add sub directory if exists

VPATH   = api handlers
SUBDIRS =
DEPPATH = --dep-path api
DEPPATH += --dep-path handlers

CSRCS  = mbedtlsstub_utils.c
CSRCS  += mbedtlsstub_ssl.c
CSRCS  += mbedtlsstub_ssl_conf.c
CSRCS  += mbedtlsstub_ssl_session.c
CSRCS  += mbedtlsstub_x509_crt.c
CSRCS  += mbedtlsstub_pk.c
CSRCS  += mbedtlsstub_ctr_drbg.c
CSRCS  += mbedtlsstub_entropy.c
CSRCS  += mbedtlsstub_cipher.c
CSRCS  += mbedtlsstub_md.c
CSRCS  += mbedtlsstub_base64.c
CSRCS  += mbedtlsstub_sha1.c
CSRCS  += mbedtlsstub_bignum.c
CSRCS  += mbedtlsstub_x509_csr.c
CSRCS  += mbedtlsstub_x509write_crt.c
CSRCS  += mbedtlsstub_rsa.c
CSRCS  += mbedtlsstub_ssl_srtp.c
CSRCS  += mbedtlsstub_x509_dn.c
CSRCS  += net_sockets.c
CSRCS  += mbedtlsstub_hdlr.c
CSRCS  += mbedtlsstub_hdlr_ssl.c
CSRCS  += mbedtlsstub_hdlr_ssl_conf.c
CSRCS  += mbedtlsstub_hdlr_ssl_session.c
CSRCS  += mbedtlsstub_hdlr_x509_crt.c
CSRCS  += mbedtlsstub_hdlr_pk.c
CSRCS  += mbedtlsstub_hdlr_ctr_drbg.c
CSRCS  += mbedtlsstub_hdlr_entropy.c
CSRCS  += mbedtlsstub_hdlr_cipher.c
CSRCS  += mbedtlsstub_hdlr_md.c
CSRCS  += mbedtlsstub_hdlr_base64.c
CSRCS  += mbedtlsstub_hdlr_sha1.c
CSRCS  += mbedtlsstub_hdlr_bignum.c
CSRCS  += mbedtlsstub_hdlr_x509_csr.c
CSRCS  += mbedtlsstub_hdlr_x509write_crt.c
CSRCS  += mbedtlsstub_hdlr_rsa.c
CSRCS  += mbedtlsstub_hdlr_ssl_srtp.c
CSRCS  += mbedtlsstub_hdlr_x509_dn.c
CXXSRCS =

ifeq ($(CONFIG_LTE_NET_MBEDTLS),y)
ifeq ($(CONFIG_EXTERNALS_MBEDTLS),y)
ifeq ($(filter clean distclean,$(MAKECMDGOALS)),)
$(error "Configs LTE_NET_MBEDTLS and EXTERNALS_MBEDTLS are exclusive. \
        So disable one or the other")
endif
endif
endif

include $(SDKDIR)/modules/Module.mk
