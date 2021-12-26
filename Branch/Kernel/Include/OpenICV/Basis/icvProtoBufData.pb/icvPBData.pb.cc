// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: icvPBData.proto

#include "icvPBData.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG
namespace icvPBDataPkg {
constexpr icvPBDataMsg::icvPBDataMsg(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : buffer_(nullptr)
  , len_(0u)
  , timestamp_(0){}
struct icvPBDataMsgDefaultTypeInternal {
  constexpr icvPBDataMsgDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~icvPBDataMsgDefaultTypeInternal() {}
  union {
    icvPBDataMsg _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT icvPBDataMsgDefaultTypeInternal _icvPBDataMsg_default_instance_;
constexpr icvTestPbMsg::icvTestPbMsg(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : name_(&::PROTOBUF_NAMESPACE_ID::internal::fixed_address_empty_string)
  , id_(0){}
struct icvTestPbMsgDefaultTypeInternal {
  constexpr icvTestPbMsgDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~icvTestPbMsgDefaultTypeInternal() {}
  union {
    icvTestPbMsg _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT icvTestPbMsgDefaultTypeInternal _icvTestPbMsg_default_instance_;
}  // namespace icvPBDataPkg
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_icvPBData_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_icvPBData_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_icvPBData_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_icvPBData_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::icvPBDataPkg::icvPBDataMsg, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::icvPBDataPkg::icvPBDataMsg, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::icvPBDataPkg::icvPBDataMsg, len_),
  PROTOBUF_FIELD_OFFSET(::icvPBDataPkg::icvPBDataMsg, timestamp_),
  PROTOBUF_FIELD_OFFSET(::icvPBDataPkg::icvPBDataMsg, buffer_),
  1,
  2,
  0,
  PROTOBUF_FIELD_OFFSET(::icvPBDataPkg::icvTestPbMsg, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::icvPBDataPkg::icvTestPbMsg, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::icvPBDataPkg::icvTestPbMsg, id_),
  PROTOBUF_FIELD_OFFSET(::icvPBDataPkg::icvTestPbMsg, name_),
  1,
  0,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(::icvPBDataPkg::icvPBDataMsg)},
  { 11, 18, sizeof(::icvPBDataPkg::icvTestPbMsg)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::icvPBDataPkg::_icvPBDataMsg_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::icvPBDataPkg::_icvTestPbMsg_default_instance_),
};

const char descriptor_table_protodef_icvPBData_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\017icvPBData.proto\022\014icvPBDataPkg\032\031google/"
  "protobuf/any.proto\"\204\001\n\014icvPBDataMsg\022\020\n\003l"
  "en\030\001 \001(\rH\000\210\001\001\022\026\n\ttimestamp\030\002 \001(\021H\001\210\001\001\022)\n"
  "\006buffer\030\003 \001(\0132\024.google.protobuf.AnyH\002\210\001\001"
  "B\006\n\004_lenB\014\n\n_timestampB\t\n\007_buffer\"B\n\014icv"
  "TestPbMsg\022\017\n\002id\030\002 \001(\005H\000\210\001\001\022\021\n\004name\030\001 \001(\t"
  "H\001\210\001\001B\005\n\003_idB\007\n\005_nameb\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_icvPBData_2eproto_deps[1] = {
  &::descriptor_table_google_2fprotobuf_2fany_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_icvPBData_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_icvPBData_2eproto = {
  false, false, 269, descriptor_table_protodef_icvPBData_2eproto, "icvPBData.proto", 
  &descriptor_table_icvPBData_2eproto_once, descriptor_table_icvPBData_2eproto_deps, 1, 2,
  schemas, file_default_instances, TableStruct_icvPBData_2eproto::offsets,
  file_level_metadata_icvPBData_2eproto, file_level_enum_descriptors_icvPBData_2eproto, file_level_service_descriptors_icvPBData_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK ::PROTOBUF_NAMESPACE_ID::Metadata
descriptor_table_icvPBData_2eproto_metadata_getter(int index) {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_icvPBData_2eproto);
  return descriptor_table_icvPBData_2eproto.file_level_metadata[index];
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_icvPBData_2eproto(&descriptor_table_icvPBData_2eproto);
namespace icvPBDataPkg {

// ===================================================================

class icvPBDataMsg::_Internal {
 public:
  using HasBits = decltype(std::declval<icvPBDataMsg>()._has_bits_);
  static void set_has_len(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_timestamp(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static const PROTOBUF_NAMESPACE_ID::Any& buffer(const icvPBDataMsg* msg);
  static void set_has_buffer(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

const PROTOBUF_NAMESPACE_ID::Any&
icvPBDataMsg::_Internal::buffer(const icvPBDataMsg* msg) {
  return *msg->buffer_;
}
void icvPBDataMsg::clear_buffer() {
  if (GetArena() == nullptr && buffer_ != nullptr) {
    delete buffer_;
  }
  buffer_ = nullptr;
  _has_bits_[0] &= ~0x00000001u;
}
icvPBDataMsg::icvPBDataMsg(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:icvPBDataPkg.icvPBDataMsg)
}
icvPBDataMsg::icvPBDataMsg(const icvPBDataMsg& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_buffer()) {
    buffer_ = new PROTOBUF_NAMESPACE_ID::Any(*from.buffer_);
  } else {
    buffer_ = nullptr;
  }
  ::memcpy(&len_, &from.len_,
    static_cast<size_t>(reinterpret_cast<char*>(&timestamp_) -
    reinterpret_cast<char*>(&len_)) + sizeof(timestamp_));
  // @@protoc_insertion_point(copy_constructor:icvPBDataPkg.icvPBDataMsg)
}

void icvPBDataMsg::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&buffer_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&timestamp_) -
    reinterpret_cast<char*>(&buffer_)) + sizeof(timestamp_));
}

icvPBDataMsg::~icvPBDataMsg() {
  // @@protoc_insertion_point(destructor:icvPBDataPkg.icvPBDataMsg)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void icvPBDataMsg::SharedDtor() {
  GOOGLE_DCHECK(GetArena() == nullptr);
  if (this != internal_default_instance()) delete buffer_;
}

void icvPBDataMsg::ArenaDtor(void* object) {
  icvPBDataMsg* _this = reinterpret_cast< icvPBDataMsg* >(object);
  (void)_this;
}
void icvPBDataMsg::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void icvPBDataMsg::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void icvPBDataMsg::Clear() {
// @@protoc_insertion_point(message_clear_start:icvPBDataPkg.icvPBDataMsg)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    if (GetArena() == nullptr && buffer_ != nullptr) {
      delete buffer_;
    }
    buffer_ = nullptr;
  }
  if (cached_has_bits & 0x00000006u) {
    ::memset(&len_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&timestamp_) -
        reinterpret_cast<char*>(&len_)) + sizeof(timestamp_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* icvPBDataMsg::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // uint32 len = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          _Internal::set_has_len(&has_bits);
          len_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint32(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // sint32 timestamp = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          _Internal::set_has_timestamp(&has_bits);
          timestamp_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarintZigZag32(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // .google.protobuf.Any buffer = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          ptr = ctx->ParseMessage(_internal_mutable_buffer(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag,
            _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
            ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* icvPBDataMsg::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:icvPBDataPkg.icvPBDataMsg)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // uint32 len = 1;
  if (_internal_has_len()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt32ToArray(1, this->_internal_len(), target);
  }

  // sint32 timestamp = 2;
  if (_internal_has_timestamp()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteSInt32ToArray(2, this->_internal_timestamp(), target);
  }

  // .google.protobuf.Any buffer = 3;
  if (_internal_has_buffer()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        3, _Internal::buffer(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:icvPBDataPkg.icvPBDataMsg)
  return target;
}

size_t icvPBDataMsg::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:icvPBDataPkg.icvPBDataMsg)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    // .google.protobuf.Any buffer = 3;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *buffer_);
    }

    // uint32 len = 1;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt32Size(
          this->_internal_len());
    }

    // sint32 timestamp = 2;
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SInt32Size(
          this->_internal_timestamp());
    }

  }
  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void icvPBDataMsg::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:icvPBDataPkg.icvPBDataMsg)
  GOOGLE_DCHECK_NE(&from, this);
  const icvPBDataMsg* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<icvPBDataMsg>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:icvPBDataPkg.icvPBDataMsg)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:icvPBDataPkg.icvPBDataMsg)
    MergeFrom(*source);
  }
}

void icvPBDataMsg::MergeFrom(const icvPBDataMsg& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:icvPBDataPkg.icvPBDataMsg)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_buffer()->PROTOBUF_NAMESPACE_ID::Any::MergeFrom(from._internal_buffer());
    }
    if (cached_has_bits & 0x00000002u) {
      len_ = from.len_;
    }
    if (cached_has_bits & 0x00000004u) {
      timestamp_ = from.timestamp_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void icvPBDataMsg::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:icvPBDataPkg.icvPBDataMsg)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void icvPBDataMsg::CopyFrom(const icvPBDataMsg& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:icvPBDataPkg.icvPBDataMsg)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool icvPBDataMsg::IsInitialized() const {
  return true;
}

void icvPBDataMsg::InternalSwap(icvPBDataMsg* other) {
  using std::swap;
  _internal_metadata_.Swap<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(icvPBDataMsg, timestamp_)
      + sizeof(icvPBDataMsg::timestamp_)
      - PROTOBUF_FIELD_OFFSET(icvPBDataMsg, buffer_)>(
          reinterpret_cast<char*>(&buffer_),
          reinterpret_cast<char*>(&other->buffer_));
}

::PROTOBUF_NAMESPACE_ID::Metadata icvPBDataMsg::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

class icvTestPbMsg::_Internal {
 public:
  using HasBits = decltype(std::declval<icvTestPbMsg>()._has_bits_);
  static void set_has_id(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_name(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

icvTestPbMsg::icvTestPbMsg(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:icvPBDataPkg.icvTestPbMsg)
}
icvTestPbMsg::icvTestPbMsg(const icvTestPbMsg& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  name_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from._internal_has_name()) {
    name_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_name(), 
      GetArena());
  }
  id_ = from.id_;
  // @@protoc_insertion_point(copy_constructor:icvPBDataPkg.icvTestPbMsg)
}

void icvTestPbMsg::SharedCtor() {
name_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
id_ = 0;
}

icvTestPbMsg::~icvTestPbMsg() {
  // @@protoc_insertion_point(destructor:icvPBDataPkg.icvTestPbMsg)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void icvTestPbMsg::SharedDtor() {
  GOOGLE_DCHECK(GetArena() == nullptr);
  name_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

void icvTestPbMsg::ArenaDtor(void* object) {
  icvTestPbMsg* _this = reinterpret_cast< icvTestPbMsg* >(object);
  (void)_this;
}
void icvTestPbMsg::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void icvTestPbMsg::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void icvTestPbMsg::Clear() {
// @@protoc_insertion_point(message_clear_start:icvPBDataPkg.icvTestPbMsg)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    name_.ClearNonDefaultToEmpty();
  }
  id_ = 0;
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* icvTestPbMsg::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // string name = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          auto str = _internal_mutable_name();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "icvPBDataPkg.icvTestPbMsg.name"));
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // int32 id = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          _Internal::set_has_id(&has_bits);
          id_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag,
            _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
            ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* icvTestPbMsg::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:icvPBDataPkg.icvTestPbMsg)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // string name = 1;
  if (_internal_has_name()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::VerifyUtf8String(
      this->_internal_name().data(), static_cast<int>(this->_internal_name().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SERIALIZE,
      "icvPBDataPkg.icvTestPbMsg.name");
    target = stream->WriteStringMaybeAliased(
        1, this->_internal_name(), target);
  }

  // int32 id = 2;
  if (_internal_has_id()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(2, this->_internal_id(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:icvPBDataPkg.icvTestPbMsg)
  return target;
}

size_t icvTestPbMsg::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:icvPBDataPkg.icvTestPbMsg)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    // string name = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->_internal_name());
    }

    // int32 id = 2;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
          this->_internal_id());
    }

  }
  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void icvTestPbMsg::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:icvPBDataPkg.icvTestPbMsg)
  GOOGLE_DCHECK_NE(&from, this);
  const icvTestPbMsg* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<icvTestPbMsg>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:icvPBDataPkg.icvTestPbMsg)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:icvPBDataPkg.icvTestPbMsg)
    MergeFrom(*source);
  }
}

void icvTestPbMsg::MergeFrom(const icvTestPbMsg& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:icvPBDataPkg.icvTestPbMsg)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      _internal_set_name(from._internal_name());
    }
    if (cached_has_bits & 0x00000002u) {
      id_ = from.id_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void icvTestPbMsg::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:icvPBDataPkg.icvTestPbMsg)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void icvTestPbMsg::CopyFrom(const icvTestPbMsg& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:icvPBDataPkg.icvTestPbMsg)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool icvTestPbMsg::IsInitialized() const {
  return true;
}

void icvTestPbMsg::InternalSwap(icvTestPbMsg* other) {
  using std::swap;
  _internal_metadata_.Swap<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  name_.Swap(&other->name_, &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
  swap(id_, other->id_);
}

::PROTOBUF_NAMESPACE_ID::Metadata icvTestPbMsg::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace icvPBDataPkg
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::icvPBDataPkg::icvPBDataMsg* Arena::CreateMaybeMessage< ::icvPBDataPkg::icvPBDataMsg >(Arena* arena) {
  return Arena::CreateMessageInternal< ::icvPBDataPkg::icvPBDataMsg >(arena);
}
template<> PROTOBUF_NOINLINE ::icvPBDataPkg::icvTestPbMsg* Arena::CreateMaybeMessage< ::icvPBDataPkg::icvTestPbMsg >(Arena* arena) {
  return Arena::CreateMessageInternal< ::icvPBDataPkg::icvTestPbMsg >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
